#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
#
# Authors: Darby Lim, Pyo

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration('port', default='/dev/ttyUSB0')

    frame_id = LaunchConfiguration('frame_id', default='laser')
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([

        DeclareLaunchArgument(
            'port',
            default_value=port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar. Default frame_id is \'laser\''),

        Node(
            package='hls_lfcd_lds_driver',
            node_executable='hlds_laser_publisher',
            node_namespace=namespace,
            node_name='hlds_laser_publisher',
            parameters=[{'port': port, 'frame_id': frame_id}],
            output='screen'),
    ])
