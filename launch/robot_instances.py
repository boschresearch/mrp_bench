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

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import launch
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from dotmap import DotMap



# opaque function. Another example for such a function can be found at
# https://github.com/jrgnicho/collaborative-robotic-sanding/blob/3902e4f0e76bde226b18a997fd60fc30e1961212/crs_application/launch/perception.launch.py#L21
def launch_setup(context, *args, **kwargs):
 
    startIndex = launch.substitutions.LaunchConfiguration('startIndex').perform(context)
    endIndex = launch.substitutions.LaunchConfiguration('endIndex').perform(context)

    print("startIndex:", startIndex)
    print("endIndex:", endIndex)


    # config
    config_path = os.path.join(get_package_share_directory('bench_pkg'), 'param/config.yaml')

    with open(config_path, 'r') as stream:
        try:
            config = DotMap(yaml.safe_load(stream))
        except yaml.YAMLError as exc:
            print(exc)
            return

    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    
    # for bringing in external components:
    full_path = os.path.realpath(__file__)
    this_script_dir = os.path.dirname(full_path)

    robots_description_path = config.common.pathToRobotsList

    with open(robots_description_path, 'r') as stream:
        try:
            robots = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            return

    # for staggered launch
    robots = robots[int(startIndex):int(endIndex)]

    # On this example all robots are launched with the same settings
    autostart = LaunchConfiguration('autostart')
    rviz_config_file = LaunchConfiguration('rviz_config')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    log_settings = LaunchConfiguration('log_settings', default='true')


    map_yaml_file = os.path.join(this_script_dir, 'mrp_bench_maps/maps', config.common.mapName, config.common.mapName + '_map.yaml')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=map_yaml_file,
        description='Full path to map file to load')

    # create temporary yaml for each robot with parameters overwritten where necessary
    base_param_file = os.path.join(this_script_dir, 'modified_standard_launch_scripts', 'turtlebot3_burger_custom.yaml')
    port_start_value = 1667 + int(startIndex) * 100 # WARNING: This allowes 50 robots per launch instance max. But that should be enough for now...
    robot_params_cmds = []
    fleet_client_nodes = []
    static_tfs = []

    for counter, robot in enumerate(robots):
        # create modified yaml from baseline
        tmp_path = '/tmp/robot_' + robot['name'] + '_params.yaml'
        try:
            with open(base_param_file, "r") as stream:
                try:
                    y = yaml.safe_load(stream)
                except yaml.YAMLError as exc:
                    print(exc)
                y['amcl']['ros__parameters']['initial_pose']['x'] = float(robot['x_pose'])
                y['amcl']['ros__parameters']['initial_pose']['y'] = float(robot['y_pose'])
                y['bt_navigator']['ros__parameters']['groot_zmq_publisher_port'] = port_start_value + counter * 2
                y['bt_navigator']['ros__parameters']['groot_zmq_server_port'] = port_start_value + 1 + counter * 2
                y['bt_navigator']['ros__parameters']['odom_topic'] = robot['name'] + '/odom'
                y['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']['scan']['topic'] = '/' + robot['name'] + '/scan'
                y['local_costmap']['local_costmap']['ros__parameters']['voxel_layer']['scan']['topic'] = '/' + robot['name'] + '/scan'
                y['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']['scan']['topic'] = '/' + robot['name'] + '/scan'
                y['global_costmap']['global_costmap']['ros__parameters']['voxel_layer']['scan']['topic'] = '/' + robot['name'] + '/scan'
                try:
                    with open(tmp_path, 'w') as f:
                        yaml.dump(y, f, sort_keys=False, default_flow_style=False) 
                except EnvironmentError as err:
                    print(err)
        except EnvironmentError as err:
            print(err)
    
        # add cmd launch param
        robot_params_cmds.append(
            DeclareLaunchArgument(
                'robot_' + robot['name'] + '_params_file',
                default_value=tmp_path,
                description='Full path to the ROS2 parameters file to use for this robot\'s launched nodes')
            )

        # add cmd for fleet client
        client_node = Node(
        package='free_fleet_client_ros2',
        executable='free_fleet_client_ros2',
        name='free_fleet_client_' + robot['name'],
        parameters=[
            {'fleet_name': config.common.fleet.name},
            {'robot_name': robot['name']},
            {'robot_model': 'turtlebot3'},
            {'level_name': config.common.levelName},
            {'dds_domain': 42},
            {'max_dist_to_first_waypoint': 10.0},
            {'map_frame': 'map'},
            {'robot_frame': robot['name'] + '/base_footprint'},
            {'nav2_server_name': robot['name'] + '/navigate_to_pose'},
            {'use_sim_time': True},
            {'namespace': robot['name']},
            {'update_frequency': 20.0},
            {'publish_frequency': 2.0},
            {'wait_timeout': 60.0},
            ],
        output='both',
        emulate_tty=True,
        )

        fleet_client_nodes.append(client_node)


        # static transform for localization
        static_transform_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_odom_' + robot['name'],
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', robot['name'] + '/odom']
        )
        
        static_tfs.append(static_transform_map_odom)
        


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

    # Define commands for spawing the robots into Gazebo
    spawn_robots_cmds = []
    for robot in robots:
        spawn_robots_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch',
                                                           'spawn_tb3_launch.py')),
                launch_arguments={
                                  'x_pose': TextSubstitution(text=str(robot['x_pose'])),
                                  'y_pose': TextSubstitution(text=str(robot['y_pose'])),
                                  'z_pose': TextSubstitution(text=str(robot['z_pose'])),
                                  'robot_name': robot['name'],
                                  'turtlebot_type': TextSubstitution(text='burger')
                                  }.items()))

    # Define commands for launching the navigation instances
    nav_instances_cmds = []
    for robot in robots:
        params_file = LaunchConfiguration(f"robot_{robot['name']}_params_file")

        group = GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, 'rviz_launch.py')),
                condition=IfCondition(use_rviz),
                launch_arguments={
                                  'namespace': TextSubstitution(text=robot['name']),
                                  'use_namespace': 'True',
                                  'rviz_config': rviz_config_file}.items()),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(this_script_dir,
                                                            'modified_standard_launch_scripts',
                                                           'tb3_simulation_launch.py')),
                # PythonLaunchDescriptionSource(os.path.join(bringup_dir,
                #                                            'launch',
                                                        #    'tb3_simulation_launch.py')),
                launch_arguments={'namespace': robot['name'],
                                  'use_namespace': 'True',
                                  'map': map_yaml_file,
                                  'use_sim_time': 'True',
                                  'params_file': params_file,
                                  'autostart': autostart,
                                  'use_rviz': 'False',
                                  'use_simulator': 'False',
                                  'headless': str(config.common.headless),
                                  'use_robot_state_pub': use_robot_state_pub}.items()),

            LogInfo(
                condition=IfCondition(log_settings),
                msg=['Launching ', robot['name']]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' params yaml: ', params_file]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' rviz config file: ', rviz_config_file]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' using robot state pub: ', use_robot_state_pub]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' autostart: ', autostart])
        ])

        nav_instances_cmds.append(group)

    # Create the launch description as list due to opque construction
    ld = []

    # Declare the launch options
    ld.append(declare_map_yaml_cmd)

        
    for node in static_tfs:
        ld.append(node)

    for node in fleet_client_nodes:
        ld.append(node)

    for cmd in robot_params_cmds:
        ld.append(cmd)

    ld.append(declare_use_rviz_cmd)
    ld.append(declare_sim_time_cmd)
    ld.append(declare_autostart_cmd)
    ld.append(declare_rviz_config_file_cmd)
    ld.append(declare_use_robot_state_pub_cmd)

    for spawn_robot_cmd in spawn_robots_cmds:
        ld.append(spawn_robot_cmd)

    for nav_inst in nav_instances_cmds:
        ld.append(nav_inst)

    return ld


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('startIndex'),
        launch.actions.DeclareLaunchArgument('endIndex'),
        OpaqueFunction(function = launch_setup)
        ])
