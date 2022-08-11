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

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
import os
from ament_index_python.packages import get_package_share_directory
import yaml
from dotmap import DotMap

config_path = os.path.join(get_package_share_directory('bench_pkg'), 'param/config.yaml')

with open(config_path, 'r') as stream:
    try:
        config = DotMap(yaml.safe_load(stream))
    except yaml.YAMLError as exc:
        print(exc)


def generate_launch_description():

    # for bringing in external components:
    full_path = os.path.realpath(__file__)
    this_script_dir = os.path.dirname(full_path)

    ld = LaunchDescription([

    DeclareLaunchArgument(
        'map_name', default_value=config.common.mapName,
        description='Map like office, airport_terminal, ...'
    ),

    DeclareLaunchArgument(
        'map_package', default_value='mrp_bench_maps',
        description='Customized maps'
    ),

    DeclareLaunchArgument(
        'headless', default_value=str(config.common.headless),
        description='gazebo headless sim flag'
    ),

    DeclareLaunchArgument(
        'use_crowdsim', default_value='True',
        description='enable crowdsim flag'
    ),

    DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use the /clock topic for time to sync with simulation'
    ),

    DeclareLaunchArgument(
        'viz_config_file', default_value=os.path.join(get_package_share_directory('rmf_demos'), 'include/' + config.common.mapName, config.common.mapName + '.rviz'),
        description='rviz config for rmf'
    ),

    DeclareLaunchArgument(
        'config_file', default_value=os.path.join(get_package_share_directory('mrp_bench_maps'), config.common.mapName, config.common.mapName + '.building.yaml'),
        description=''
    ),

    DeclareLaunchArgument(
        'dashboard_config_file', default_value=os.path.join(get_package_share_directory('rmf_demos_dashboard_resources'), config.common.mapName, 'dashboard_config.json'),
        description=''
    ),


    Node(
        package='bench_pkg',
        executable='bench_node',
        output='both',
        emulate_tty=True,
    ),

    # rmf commons
    IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(get_package_share_directory('rmf_demos'), 'common.launch.xml')),
            launch_arguments = {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'viz_config_file': LaunchConfiguration('viz_config_file'),
                'config_file': LaunchConfiguration('config_file'),
                'dashboard_config_file': LaunchConfiguration('dashboard_config_file'),
            }.items()
    ),

    # gazebo simulation
    IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(this_script_dir,
                                                'modified_standard_launch_scripts',
                                                'simulation.launch.xml')),
            launch_arguments = {
                'map_name': LaunchConfiguration('map_name'),
                'map_package': LaunchConfiguration('map_package'),
                'headless': LaunchConfiguration('headless'),
                'use_crowdsim': LaunchConfiguration('use_crowdsim')
            }.items()
    ),

    ])

    return ld