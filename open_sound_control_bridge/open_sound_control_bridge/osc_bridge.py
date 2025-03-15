#!/usr/bin/env python3
# Copyright 2025 Chris Iverach-Brereton
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory
import argparse
import datetime
import importlib
from open_sound_control_msgs.msg import OscBlob, OscMessage
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import socket
from std_msgs.msg import (
    Bool,
    ByteMultiArray,
    ColorRGBA,
    Empty,
    Float32,
    Float64,
    Header,
    Int32,
    String,
)
import struct
import threading
import yaml


epoch_ref = datetime.datetime(
    year = 1970,
    month = 1,
    day = 1,
    hour = 0,
    minute = 0,
    second = 0
)

ntp_ref = datetime.datetime(
    year = 1900,
    month = 1,
    day = 1,
    hour = 0,
    minute = 0,
    second = 0
)


def ros_time_2_ntp_time(ros_time: rclpy.time.Time) -> float:
    """
    Convert a ROS timestamp (linux epoch time) to NTP time

    @param ros_time The ROS timestamp to convert
    @return  The elapsed seconds since 1 Jan 1900 00:00:00
    """
    s, ns = ros_time.seconds_nanoseconds()
    epoch_delta = datetime.timedelta(
        seconds=(s + ns / 1000000000)
    )
    ntp_delta = (epoch_ref + epoch_delta) - ntp_ref
    return ntp_delta.total_seconds()


def ntp_time_2_ros_time(ntp_time: float) -> rclpy.time.Time:
    """
    Convert an NTP timestamp to a ROS timestamp

    @param ntp_time  The number of seconds elapsed since Jan 1, 1900 00:00:00
    """
    ntp_delta = datetime.timedelta(seconds=ntp_time)
    now = ntp_ref + ntp_delta
    epoch_delta = now - epoch_ref
    return rclpy.time.Time(
        nanoseconds = epoch_delta.total_seconds() * 1000000000
    )


class Ros2OscRelay:
    """
    Helper class that converts ROS data to OSC data

    The OscBridgeNode creates one of these for every listener topic defined in the config
    """

    def __init__(
        self,
        node: rclpy.node.Node,
        ros_topic: str,
        ros_type: any,
        osc_address: str,
        dest_ip: str,
        udp_port: int,
        osc_type: str = None,
    ):
        """
        Create the ROS to OSC relay

        @param node  The ROS node that owns this relay
        @param ros_topic  The ROS topic we're subscribing to
        @param ros_type  The ROS message type for the subscription
        @param osc_address  The OSC address we forward messages as
        @param dest_ip  The IP address of the OSC client we're sending data to
        @param udp_port  The UDP port we send the OSC packet on
        @param osc_type  Optional parameter to disambiguate between multiple OSC types
                         that translate to the same ROS type
                         (e.g. std_msgs/String, std_msgs/Empty)

        @exception ValueError if the specified osc_type is not compatible with the given
                   ROS type
        """
        self.node = node
        self.dest_ip = dest_ip
        self.udp_port = udp_port
        self.osc_address = osc_address
        self.osc_type = osc_type

        # sanity check that the osc_type is valid for our ROS type
        if osc_type is not None:
            if not (
                (osc_type == 'N' and ros_type is Empty) or
                (osc_type == 'I' and ros_type is Empty) or
                (osc_type == 's' and ros_type is String) or
                (osc_type == 'S' and ros_type is String) or
                (osc_type == 'c' and ros_type is String)
            ):
                raise ValueError(f'OSC type {osc_type} is not compatible with ROS type {ros_type}')

        self.udp_socket = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
            socket.IPPROTO_UDP,
        )
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        self.sub = node.create_subscription(
            ros_type,
            ros_topic,
            self.ros_callback,
            qos_profile_sensor_data
        )

    def ros_callback(self, data):
        """
        Callback function for receiving data on our ROS topic

        We convert the ROS data to its equivalent OSC packet and send it over
        our UDP socket to the specified destination. This being UDP there's no
        handshaking to make sure it was received; we just fire & forget.

        @param data  The ROS message we've received
        """
        osc_header = []
        osc_payload = []
        t = type(data)

        def word_align(arr):
            """
            Pad osc_data with extra null characters so it's 32-bit aligned
            """
            for i in range((4 - (len(arr) % 4)) % 4):
                arr.append(0)

        # 1. the OSC address
        if t is OscMessage:
            # use the address in the message instead of our static one
            for ch in data.address:
                osc_header.append(ord(ch))
        else:
            for ch in self.osc_address:
                osc_header.append(ord(ch))
        osc_header.append(0)
        osc_header.append(ord(','))
        word_align(osc_header)

        # 2. the type & payload
        if t is Bool:
            # Booleans have no payload data
            if data.data:
                osc_header.append(ord(OscMessage.B_TRUE))
            else:
                osc_header.append(ord(OscMessage.B_FALSE))
        elif t is Empty:
            # empty can be a trigger or null
            # neither has any payload
            if self.osc_type is None:
                osc_header.append(ord(OscMessage.NIL))
            else:
                osc_header.append(self.osc_type)
        elif t is ByteMultiArray:
            # 4-byte MIDI data
            osc_header.append(ord(OscMessage.MIDI))
            for i in range(4):
                osc_payload.append(data.data[i])
        elif t is ColorRGBA:
            osc_header.append(ord(OscMessage.RGBA))
            # ROS uses 0-1, OSC uses 0-255
            osc_payload.append(int(data.r * 255))
            osc_payload.append(int(data.g * 255))
            osc_payload.append(int(data.b * 255))
            osc_payload.append(int(data.a * 255))
        elif t is Float32:
            osc_header.append(ord(OscMessage.FLOAT))
            fbytes = struct.pack('>f', data.data)
            for b in fbytes:
                osc_payload.append(b)
        elif t is Float64:
            osc_header.append(ord(OscMessage.DOUBLE))
            fbytes = struct.pack('>d', data.data)
            for b in fbytes:
                osc_payload.append(b)
        elif t is Header:
            # OSC timestamps are a 64-bit fixed-point value
            # the first 32 bits are the seconds elapsed since 1 Jan 1900 00:00:00
            # the second 32 bits are the fractional parts of a second
            osc_header.append(ord(OscMessage.TIMETAG))
            t = ros_time_2_ntp_time(t.stamp)
            s = int(t)
            sub_s = t - s
            fix = int(sub_s * 2**32 - 1)
            osc_payload.append((s >> 24) & 0xFF)
            osc_payload.append((s >> 16) & 0xFF)
            osc_payload.append((s >> 8) & 0xFF)
            osc_payload.append(s & 0xFF)
            osc_payload.append((fix >> 24) & 0xFF)
            osc_payload.append((fix >> 16) & 0xFF)
            osc_payload.append((fix >> 8) & 0xFF)
            osc_payload.append(fix & 0xFF)
        elif t is Int32:
            osc_header.append(ord(OscMessage.INTEGER))
            osc_payload.append((data.data >> 24) & 0xFF)
            osc_payload.append((data.data >> 16) & 0xFF)
            osc_payload.append((data.data >> 8) & 0xFF)
            osc_payload.append(data.data & 0xFF)
        elif t is String:
            osc_header.append(ord(OscMessage.STRING))
            for ch in data.data:
                osc_payload.append(ord(ch))
            osc_payload.append(0)
            word_align(osc_payload)
        elif t is OscBlob:
            # OSC blob data
            osc_header.append(ord(OscMessage.BLOB))
            # data length
            osc_payload.append((len(data.data) >> 24) & 0xFF)
            osc_payload.append((len(data.data) >> 16) & 0xFF)
            osc_payload.append((len(data.data) >> 8) & 0xFF)
            osc_payload.append(len(data.data) & 0xFF)
            for byte in data.data:
                osc_payload.append(byte)
            word_align(osc_payload)
        elif t is OscMessage:
            for ch in data.types:
                osc_header.append(ord(ch))
            for b in data.payload:
                osc_payload.append(b)

        for b in osc_payload:
            osc_header.append(b)

        msg = bytearray(osc_header)
        self.udp_socket.sendto(msg, (self.dest_ip, self.udp_port))

class OscBridgeNode(Node):
    def __init__(
        self,
        config_path: str,
        udp_port: int,
    ):
        super().__init__('osc_bridge_node')
        self.config_path = config_path
        self.udp_port = udp_port

        self.parse_config()

        self.udp_socket = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
            socket.IPPROTO_UDP,
        )
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_socket.settimeout(0)
        addr = socket.getaddrinfo("0.0.0.0", self.udp_port)[0][-1]
        self.udp_socket.bind(addr)

        # keep a dictionary of OSC -> ROS republishers keyed by their ROS topic name
        self.publishers = {}

        self.is_alive = True
        self.socket_thread = threading.Thread(
            target=self.read_socket
        )
        self.socket_thread.start()

    def shutdown(self, *, context=None):
        self.is_alive = False
        self.socket_thread.join()
        super().shutdown(context=context)

    def parse_config(self) -> None:
        """Read the configuration file & create the ROS topic subscribers"""
        try:
            with open(self.config_path, 'r') as yaml_in:
                cfg = yaml.load(yaml_in, yaml.SafeLoader)
            listeners = cfg.get('listeners', [])

            self.ros_to_osc_subs = []
            for l in listeners:
                tstring = l['type']
                parts = tstring.split('/')
                module = f'{parts[0]}.{parts[1]}'
                msg_name = parts[2]
                msg_type = getattr(importlib.import_module(module), msg_name)
                topic = l['topic']
                self.get_logger().info(
                    f'Creating subscriber {topic} ({msg_type})'
                )
                try:
                    sub = Ros2OscRelay(
                        self,
                        topic,
                        msg_type,
                        l['osc_address'],
                        l['host'],
                        l['port'],
                        osc_type=l.get('osc_type', None),
                    )
                    self.ros_to_osc_subs.append(sub)
                except KeyError as err:
                    self.get_logger().warning(f'Failed to create subscriber: missing config key "{err}". Skipping.')

        except Exception as err:
            self.get_logger().error(f'Failed to read config file {self.config_path}: {err}')

    def read_socket(self):
        while self.is_alive:
            try:
                (data, connection) = self.udp_socket.recvfrom(1024)
                msg = self.osc2ros(data)
            except ValueError as err:
                pass
            except OSError as err:
                pass
            except Exception as err:
                self.get_logger().warning(f'Failed to process packet: {err}')

    def osc2ros(self, osc_packet: bytes) -> open_sound_control_msgs.msg.OscMessage:
        """
        Convert a raw OSC packet into its equivalent ROS message

        @param osc_packet  The raw byte data received from the socket
        """
        def align_next_word(n):
            """
            Return the index of the next word-alined byte

            We assume 4-byte/32-bit words. If we're already word-aligned,
            we increment to the next one

            @param n  The current index in a byte array

            @exception ValueError of the packet contains data we don't support
            """
            return n + (4 - (n % 4)) % 4

        msg = OscMessage()

        address_end = osc_packet.index(b'\0', 0)
        msg.address = osc_packet[0:address_end].decode('utf-8')
        if msg.address.endswith("/"):
            msg.address = msg.address.rstrip("/")

        type_start = osc_packet.index(b',', address_end)
        type_end = osc_packet.index(b'\0', type_start)
        data_start = align_next_word(type_end)

        i = type_start + 1
        d = data_start

        topic_counters = {
            OscMessage.FLOAT: 0,
            OscMessage.INTEGER: 0,
            OscMessage.BLOB: 0,
            OscMessage.STRING: 0,
            OscMessage.TIMETAG: 0,
            OscMessage.IMPULSE: 0,
            OscMessage.NIL: 0,
            OscMessage.B_TRUE: 0,
            OscMessage.B_FALSE: 0,
            OscMessage.CHAR: 0,
            OscMessage.DOUBLE: 0,
            OscMessage.MIDI: 0,
            OscMessage.RGBA: 0,
            OscMessage.SYMBOLS: 0,
        }
        while i < type_end and d < len(osc_packet):
            t = chr(osc_packet[i])

            ros_type = None
            ros_topic = None
            ros_value = None

            # Numerical types
            if t == OscMessage.INTEGER:
                ros_type = Int32
                ros_topic = f'{msg.address}/int_{topic_counters[t]}'
                ros_value = Int32()
                ros_value.data = (
                    (osc_packet[d] << 24) |
                    (osc_packet[d + 1] << 16) |
                    (osc_packet[d + 2] << 8) |
                    osc_packet[d + 3]
                )
                d += 4
            elif t == OscMessage.FLOAT:
                ros_type = Float32
                ros_topic = f'{msg.address}/float_{topic_counters[t]}'
                ros_value = Float32()
                ros_value = data = struct.unpack('>f', osc_packet[d:d+4])[0]
                d += 4
            elif t == OscMessage.DOUBLE:
                ros_type = Float64
                ros_topic = f'{msg.address}/double_{topic_counters[t]}'
                ros_value = Float64()
                ros_value = data = struct.unpack('>d', osc_packet[d:d+8])[0]
                d += 8

            # string types
            elif t == OscMessage.STRING:
                pass
            elif t == OscMessage.SYMBOLS:
                pass
            elif t == OscMessage.CHAR:
                pass

            # misc payloads
            elif t == OscMessage.BLOB:
                pass
            elif t == OscMessage.TIMETAG:
                pass
            elif t == OscMessage.MIDI:
                pass
            elif t == OscMessage.RGBA:
                pass
            elif t == OscMessage.BLOB:
                pass

            # non-payload types
            elif t == OscMessage.IMPULSE:
                pass
            elif t == OscMessage.NIL:
                pass
            elif t == OscMessage.B_TRUE or t == OscMessage.B_FALSE:
                pass

            # unsupported types
            elif t == OscMessage.ARR_START or t == OscMessage.ARR_STOP:
                raise ValueError('open_sound_control_bridge does not support OSC arrays')

            topic_counters[t] += 1

        return msg


def main():
    default_cfg = f'{get_package_share_directory("open_sound_control_bridge")}/config/example_config.yaml'

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-p',
        '--port',
        action='store',
        dest='port',
        type=int,
        default=9001,
        help='UDP port we accept OSC packets on (default: 9001)',
    )
    parser.add_argument(
        '-c',
        '--config',
        action='store',
        dest='config',
        type=str,
        default=default_cfg,
        help=f'Path to the OSC bridge configuration file (default: {default_cfg})',
    )
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = OscBridgeNode(args.config, args.port)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
