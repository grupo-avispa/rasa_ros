#!/usr/bin/env python3

# Copyright (c) 2025 Alberto J. Tudela Roldán
# Copyright (c) 2025 José Galeas Merchán
# Copyright (c) 2025 Grupo Avispa, DTE, Universidad de Málaga
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launches a ROS 2 node that interacts with the Rasa Open Source server."""

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Getting directories and launch-files
    rasa_ros_dir = get_package_share_directory('rasa_ros')
    default_params_file = os.path.join(rasa_ros_dir, 'params', 'default.yaml')

    # Input parameters declaration
    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    # Map these variables to arguments: can be set from the command line or a default will be used
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file with dsr agent configuration'
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='log level'
    )

    rasa_node = Node(
        package='rasa_ros',
        namespace='',
        executable='rasa_ros',
        name='rasa',
        output='screen',
        respawn_delay=2.0,
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', ['rasa:=', log_level]]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_log_level_cmd)
    # Add the actions to launch the node
    ld.add_action(rasa_node)

    return ld
