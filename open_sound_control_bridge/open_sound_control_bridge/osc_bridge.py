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
import argparse
import importlib
import socket
import struct
import threading

from ament_index_python.packages import get_package_share_directory
from open_sound_control_bridge.ntp_utils import (
    ntp_time_2_ros_time,
)
from open_sound_control_bridge.relay import (
    Ros2OscRelay
)
from open_sound_control_msgs.msg import OscBlob, OscMessage

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
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
import yaml


class OscBridgeNode(Node):
    """
    The main interface class between OSC and ROS.

    In dynamic mode, every incoming OSC packet will generate a corresponding ROS
    publisher.

    In static mode, we pre-configure the available topics and only ever publish
    on those

    :param config_path:  The path to the configuration file with incoming & outgoing topics
    :param udp_port:  The UDP port we accept incoming OSC packets on
    :param static_bridge:  Do we operate in dynamic or static mode
    """

    def __init__(
        self,
        config_path: str,
        udp_port: int = 9000,
        static_bridge: bool = False,
    ):
        super().__init__('osc_bridge_node')
        self.config_path = config_path
        self.udp_port = udp_port

        # keep a dictionary of OSC -> ROS republishers keyed by their OSC address
        self.osc_to_ros_pubs = {}

        # keep a list of all ROS -> OSC relay objects
        self.ros_to_osc_relays = []

        self.static_mode = static_bridge
        self.parse_config()

        self.udp_socket = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
            socket.IPPROTO_UDP,
        )
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_socket.settimeout(1)
        addr = socket.getaddrinfo('0.0.0.0', self.udp_port)[0][-1]
        self.udp_socket.bind(addr)

        self.raw_publisher = self.create_publisher(
            OscMessage,
            'osc_raw',
            qos_profile_sensor_data
        )

        self.is_alive = True
        self.socket_thread = threading.Thread(
            target=self.read_socket
        )
        self.socket_thread.start()

    def shutdown(self, *, context=None):
        self.is_alive = False
        self.socket_thread.join()
        super().shutdown(context=context)

    def str2msg(self, typestr: str):
        """
        Convert a ROS type name to its actual type.

        :param typestr: The ROS name of the message, e.g 'std_msgs/msg/String'
        :return: The type, or None if the type could not be parsed
        """
        try:
            parts = typestr.split('/')
            module = '.'.join(parts[0:-1])
            msg_name = parts[-1]
            return getattr(importlib.import_module(module), msg_name)
        except Exception as err:
            self.get_logger().warning(f'Failed to find type for "{typestr}": {err}')
            return None

    def parse_config(self) -> None:
        """Read the configuration file & create the ROS topic subscribers."""
        try:
            with open(self.config_path, 'r') as yaml_in:
                cfg = yaml.load(yaml_in, yaml.SafeLoader)
            ros2osc = cfg.get('topics_ros2osc', [])
            osc2ros = cfg.get('topics_osc2ros', [])

            for listener in ros2osc:
                try:
                    tstring = listener['type']
                    msg_type = self.str2msg(tstring)

                    if msg_type is None:
                        raise ValueError(f'Unknown ROS type: {tstring}')

                    topic = listener['topic']
                    osc_address = listener['osc_address']
                    host = listener['host']
                    port = listener['port']
                    self.get_logger().info(
                        f'Bridge ROS->OSC :: {topic} ({tstring}) -> {osc_address} @ {host}:{port}'
                    )

                    sub = Ros2OscRelay(
                        self,
                        topic,
                        msg_type,
                        osc_address,
                        host,
                        port,
                        osc_type=listener.get('osc_type', None),
                    )
                    self.ros_to_osc_relays.append(sub)
                except KeyError as err:
                    self.get_logger().warning(f'Failed to create subscriber: missing config key "{err}". Skipping.')  # noqa: E501
                except ValueError as err:
                    self.get_logger().warning(f'Failed to create subscriber: {err}. Skipping.')

            if self.static_mode:
                for publisher in osc2ros:
                    try:
                        tstring = publisher['type']
                        msg_type = self.str2msg(tstring)

                        if msg_type is None:
                            raise ValueError(f'Unknown ROS type: {tstring}')

                        topic = publisher['topic']
                        osc_address = publisher['osc_address']
                        self.get_logger().info(
                            f'Bridge OSC->ROS :: {osc_address} -> {topic} ({tstring})'
                        )
                        pub = self.create_publisher(
                            msg_type,
                            topic,
                            qos_profile_sensor_data,
                        )
                        self.osc_to_ros_pubs[osc_address] = pub
                    except KeyError as err:
                        self.get_logger().warning(f'Failed to create publisher: missing config key "{err}". Skipping.')  # noqa: E501
                    except ValueError as err:
                        self.get_logger().warning(f'Failed to create publisher: {err}. Skipping.')

        except Exception as err:
            self.get_logger().error(f'Failed to read config file {self.config_path}: {err}')

    def read_socket(self):
        while self.is_alive:
            try:
                (data, _) = self.udp_socket.recvfrom(4096)
                msg = self.osc2ros(data)
                if msg is not None:
                    self.raw_publisher.publish(msg)
            except ValueError as err:
                self.get_logger().warning(f'Rejected packet: {err}')
            except OSError:
                pass
            except Exception as err:
                self.get_logger().warning(f'Failed to process packet: {err}')

    def osc2ros(self, osc_packet: bytes) -> OscMessage:
        """
        Convert a raw OSC packet into its equivalent ROS message.

        :param osc_packet:  The raw byte data received from the socket
        :return: The raw OSC message to be sent on the osc_raw topic, or None
            if we're in static mode and the OSC address wasn't predefined
        """
        def align_next_word(n):
            """
            Return the index of the next word-alined byte.

            We assume 4-byte/32-bit words. If we're already word-aligned,
            we increment to the next one

            :param n:  The current index in a byte array

            :raises ValueError: of the packet contains data we don't support
            """
            return n + (4 - (n % 4)) % 4

        msg = OscMessage()

        address_end = osc_packet.index(b'\0', 0)
        msg.address = osc_packet[0:address_end].decode('utf-8')
        if msg.address.endswith('/'):
            msg.address = msg.address.rstrip('/')

        # if we're in static mode and we don't know about this topic, kick out now;
        # there's no need to process the whole packet
        if self.static_mode and msg.address not in self.osc_to_ros_pubs.keys():
            self.get_logger().warning(f'Unknown OSC address: {msg.address}')
            return None

        type_start = osc_packet.index(b',', address_end)
        type_end = osc_packet.index(b'\0', type_start)
        data_start = align_next_word(type_end)

        # fill in the raw packet
        for i in range(type_start+1, type_end):
            msg.types.append(chr(osc_packet[i]))
        d = data_start
        while d < len(osc_packet):
            msg.payload.append(osc_packet[d])
            d += 1

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
            'T_F': 0,  # shared counter for T and F values
            OscMessage.CHAR: 0,
            OscMessage.DOUBLE: 0,
            OscMessage.MIDI: 0,
            OscMessage.RGBA: 0,
            OscMessage.SYMBOLS: 0,
        }
        while i < type_end and d < len(osc_packet):
            t = chr(osc_packet[i])
            i += 1

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
                ros_value.data = struct.unpack('>f', osc_packet[d:d+4])[0]
                d += 4
            elif t == OscMessage.DOUBLE:
                ros_type = Float64
                ros_topic = f'{msg.address}/double_{topic_counters[t]}'
                ros_value = Float64()
                ros_value.data = struct.unpack('>d', osc_packet[d:d+8])[0]
                d += 8

            # string types
            elif t == OscMessage.STRING:
                ros_type = String
                ros_topic = f'{msg.address}/string_{topic_counters[t]}'
                ros_value = String()
                str_end = osc_packet.index(b'\0', d)
                ros_value.data = osc_packet[d:str_end].decode('utf-8')
                d = align_next_word(str_end)
            elif t == OscMessage.SYMBOLS:
                ros_type = String
                ros_topic = f'{msg.address}/symbol_{topic_counters[t]}'
                ros_value = String()
                str_end = osc_packet.index(b'\0', d)
                ros_value.data = osc_packet[d:str_end].decode('utf-8')
                d = align_next_word(str_end)
            elif t == OscMessage.CHAR:
                ros_type = String
                ros_topic = f'{msg.address}/char_{topic_counters[t]}'
                ros_value = String()
                # char data is in the least significant byte of the 32-bit word
                ros_value.data = chr(osc_packet[d + 3])
                d += 4

            # misc payloads
            elif t == OscMessage.BLOB:
                ros_type = OscBlob
                ros_topic = f'{msg.address}/blob_{topic_counters[t]}'
                ros_value = OscBlob()
                n = (
                    (osc_packet[d] << 24) |
                    (osc_packet[d + 1] << 16) |
                    (osc_packet[d + 2] << 8) |
                    osc_packet[d + 3]
                )
                d += 4
                ros_value.data = []
                for j in range(n):
                    ros_value.data.append(osc_packet[d + j])
                d += n
                d = align_next_word(d)
            elif t == OscMessage.TIMETAG:
                s = (
                    (osc_packet[d] << 24) |
                    (osc_packet[d + 1] << 16) |
                    (osc_packet[d + 2] << 8) |
                    osc_packet[d + 3]
                )
                sub_s = (
                    (osc_packet[d + 4] << 24) |
                    (osc_packet[d + 5] << 16) |
                    (osc_packet[d + 6] << 8) |
                    osc_packet[d + 7]
                )
                time = s + sub_s / (2 ** 32 - 1)

                ros_type = Header
                ros_topic = f'{msg.address}/time_{topic_counters[t]}'
                ros_value = Header()
                ros_value.stamp = ntp_time_2_ros_time(time).to_msg()
                d += 8
            elif t == OscMessage.MIDI:
                ros_type = ByteMultiArray
                ros_topic = f'{msg.address}/midi_{topic_counters[t]}'
                ros_value = ByteMultiArray()
                ros_value.data = [
                    osc_packet[d],
                    osc_packet[d + 1],
                    osc_packet[d + 2],
                    osc_packet[d + 3],
                ]
                d += 4
            elif t == OscMessage.RGBA:
                ros_type = ColorRGBA
                ros_topic = f'{msg.address}/rgba_{topic_counters[t]}'
                ros_value = ColorRGBA()
                ros_value.r = osc_packet[d] / 255
                ros_value.g = osc_packet[d + 1] / 255
                ros_value.b = osc_packet[d + 2] / 255
                ros_value.a = osc_packet[d + 3] / 255
                d += 4

            # non-payload types
            elif t == OscMessage.IMPULSE:
                ros_type = Empty
                ros_topic = f'{msg.address}/trig_{topic_counters[t]}'
                ros_value = Empty()
            elif t == OscMessage.NIL:
                ros_type = Empty
                ros_topic = f'{msg.address}/null_{topic_counters[t]}'
                ros_value = Empty()
            elif t == OscMessage.B_TRUE or t == OscMessage.B_FALSE:
                ros_type = Bool
                # use a shared counter for both T and F
                ros_topic = f'{msg.address}/bool_{topic_counters["T_F"]}'
                ros_value = Bool()
                if t == OscMessage.B_TRUE:
                    ros_value.data = True
                else:
                    ros_value.data = False
                t = 'T_F'  # hack so the counter increment below works properly

            # unsupported types
            elif t == OscMessage.ARR_START or t == OscMessage.ARR_STOP:
                raise ValueError('open_sound_control_bridge does not support OSC arrays')

            topic_counters[t] += 1

            ros_topic = self.sanitize_ros_topic(ros_topic)

            # if we're here it's either because we're in dynamic mode or the address is known
            # no need to handle static mode as that's taken care of earlier
            if msg.address not in self.osc_to_ros_pubs.keys():
                self.osc_to_ros_pubs[msg.address] = self.create_publisher(
                    ros_type,
                    ros_topic,
                    qos_profile_sensor_data,
                )

            pub = self.osc_to_ros_pubs[msg.address]
            try:
                # if the OSC message has an unexpected type, the publish might fail
                pub.publish(ros_value)
            except Exception as err:
                self.get_logger().debug(f'Failed to publish OSC->ROS message: {err}')

            # if we're in static mode, kick out; we only support 1 type per packet
            if self.static_mode:
                break

        return msg

    def sanitize_ros_topic(self, t: str):
        """
        Sanitize the generated ROS topic.

        OSC allows leading integers, non-letter characters, etc... that are not compatible
        with ROS.

        :param t:  The ROS topic generated from the OSC address
        """
        namespaces = t.split('/')
        ns = []
        for n in namespaces:
            # remove non-alphanumeric characters, make everything lower-case
            s = ''
            for ch in n:
                ch = ch.lower()
                if not ch.isalnum():
                    ch = '_'
                s += ch

            if len(s) > 0:
                # ROS doesn't allow leading integers, so add an `osc_` prefix
                # if necessary; adding `_` alone would result in a hidden topic,
                # which we probably don't want
                if s[0] >= '0' and s[0] <= '9':
                    s = f'osc_{s}'
                ns.append(s)

        return '/'.join(ns)


def main():
    default_cfg = f'{get_package_share_directory("open_sound_control_bridge")}/config/example_config.yaml'  # noqa: E501

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
    parser.add_argument(
        '-s',
        '--static',
        action='store_true',
        dest='static_bridge',
        help='Enable static OSC to ROS topics; only topics in the config file will be bridged',
    )
    args, _ = parser.parse_known_args()

    rclpy.init()
    executor = MultiThreadedExecutor()
    node = OscBridgeNode(
        args.config,
        udp_port=args.port,
        static_bridge=args.static_bridge
    )
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
