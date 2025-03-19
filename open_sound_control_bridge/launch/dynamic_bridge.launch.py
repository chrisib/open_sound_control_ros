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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument(
        'osc_config',
        default_value=f'{get_package_share_directory("open_sound_control_bridge")}/config/europi_config.yaml',  # noqa: E501
        description='Path to the OSC bridge configuration file',
    ),
    DeclareLaunchArgument(
        'port',
        default_value='9001',
        description='UDP port we accept incoming OSC packets on',
    ),
]


def generate_launch_description():
    config_file = LaunchConfiguration('osc_config')
    port = LaunchConfiguration('port')

    osc_bridge_node = Node(
        package='open_sound_control_bridge',
        executable='osc_bridge_node',
        name='osc_bridge',
        output='screen',
        arguments=[
            '--config',
            config_file,
            '--port',
            port,
        ],
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(osc_bridge_node)
    return ld
