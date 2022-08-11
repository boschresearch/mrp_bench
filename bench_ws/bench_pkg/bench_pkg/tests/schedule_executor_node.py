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


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class ScheduleExecutor(Node):
    def __init__(self):
        super(ScheduleExecutor, self).__init__('schedule_executor')
        
        # create the subs
        qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
        depth=1
        )


        # Schedule Sub
        graph_sub = self.create_subscription(MarkerArray, topic='/map_markers', qos_profile=qos_profile, callback=self.graph_callback) 
        # Grid sub - grid generated and published by custom gazebo plugin
        grid_sub = self.create_subscription(OccupancyGrid, topic='/grid', qos_profile=qos_profile, callback=self.grid_callback) 
        print('Subscribers created')

        # check subs status
        print('Wait for subscriptions to be saved')


#   navigation_action_client_ =
#     rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
#     client_node_,
#     "navigate_to_pose");
#   waypoint_follower_action_client_ =
#     rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
#     client_node_,
#     "follow_waypoints");
#   navigation_goal_ = nav2_msgs::action::NavigateToPose::Goal();
#   waypoint_follower_goal_ = nav2_msgs::action::FollowWaypoints::Goal();



def main(args=None):
    print('Start')
    rclpy.init(args=args)

    se = ScheduleExecutor()

    while rclpy.ok(): # and not done:
        try:
            rclpy.spin_once(se)
        except KeyboardInterrupt:
            break

    se.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
