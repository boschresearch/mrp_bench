# Copyright (c) 2022 - for information on the respective copyright owner
# see the NOTICE file or the repository https://github.com/boschresearch/mrp-bench.
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

import os
import yaml
import time

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import launch
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from dotmap import DotMap
import glob

def generate_launch_description():
 


    # Names and poses of the robots
    config_path = os.path.join(get_package_share_directory('bench_pkg'), 'param/config.yaml')

    with open(config_path, 'r') as stream:
        try:
            config = DotMap(yaml.safe_load(stream))
        except yaml.YAMLError as exc:
            print(exc)
            return

    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')

    robots_description_path = config.common.pathToRobotsList

    with open(robots_description_path, 'r') as stream:
        try:
            robots = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            return

    # add fleet server cmd
    free_fleet_server_node = Node(
    package='free_fleet_server_ros2',
    executable='free_fleet_server_ros2',
    name='free_fleet_server_node_turtlebot3_2',
    parameters=[
        {'fleet_name': 'turtlebot3'},
        {'fleet_state_topic': 'fleet_states'},
        {'mode_request_topic': 'robot_mode_requests'},
        {'path_request_topic': config.common.fleet.pathRequestTopicName},
        {'destination_request_topic': 'robot_destination_requests'},
        {'use_sim_time': True},
        {'dds_domain': 42},
        {'dds_robot_state_topic': 'robot_state'},
        {'dds_mode_request_topic': 'mode_request'},
        {'dds_path_request_topic': 'path_request'},
        {'dds_destination_request_topic': 'destination_request'},
        {'update_state_frequency': 20.0},
        {'publish_state_frequency': 2.0},
        {'translation_x': 0.0},
        {'translation_y': 0.0},
        {'rotation': 0.0},
        {'scale': 1.0},
        ],
    output='both',
    emulate_tty=True,
    )

   
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the stacks')

    declare_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_dir, 'rviz', 'nav2_namespaced_view.rviz'),
        description='Full path to the RVIZ config file to use.')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value=str(config.common.rvizForEachAgent),
        description='Whether to start RVIZ')

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(free_fleet_server_node)


    # Add the actions to start gazebo, robots and simulations
    # ld.add_action(start_gazebo_cmd)
    return ld
