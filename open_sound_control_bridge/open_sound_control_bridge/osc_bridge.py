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
import rclpy
from rclpy.node import Node

class OscBridgeNode(Node):
    def __init__(
        self,
        config_path: str,
        udp_port: int,
    ):
        super().__init__('osc_bridge_node')
        self.config_path = config_path
        self.udp_port = udp_port


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
