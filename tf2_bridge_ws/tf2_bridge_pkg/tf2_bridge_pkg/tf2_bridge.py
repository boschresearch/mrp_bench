# Copyright 2016 Open Source Robotics Foundation, Inc.
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

from hashlib import new
from time import time

from matplotlib.transforms import Transform
import rclpy
from rclpy.node import Node
from functools import partial

from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from copy import deepcopy
from ament_index_python.packages import get_package_share_directory
import os
import yaml
from dotmap import DotMap

class TF2Subscriber(Node):

    def __init__(self, prefixes):
        super().__init__('tf2_bridge')

        self.publisher = self.create_publisher(TFMessage, 'tf', 10)
        self.subs = []
        self.sec_cleaned = {}
        self.seen = {}
        self.agent_publishers = {}


        # because lambdas are late-binding...
        def create_lambda(prefix):
            return lambda msg: self.listener_callback(msg, prefix)

        for prefix in prefixes:
            self.agent_publishers[prefix] = self.create_publisher(TFMessage, prefix +'/tf', 10)
            self.subs.append(self.create_subscription(
                TFMessage,
                prefix + '/tf',
                create_lambda(prefix),
                10))
            self.get_logger().info('Listening to ' + self.subs[-1].topic_name)  

            self.seen[prefix] = set()
            self.sec_cleaned[prefix] = 0


        self.get_logger().info('Initialized ' + self.get_name())

    def listener_callback(self, msg, prefix):
        # check if message was processed already
        timestamp = str(msg.transforms[0].header.stamp.sec) + '_' + str(msg.transforms[0].header.stamp.nanosec)
        if timestamp in self.seen[prefix]:
            return
        else:
            # clean set occasionally
            if msg.transforms[0].header.stamp.sec > self.sec_cleaned[prefix] + 10:
                self.seen[prefix] = set()
            self.seen[prefix].add(timestamp)

        # transform from namespace to global tf
        for transform in msg.transforms:
            if 'map' in transform.header.frame_id:
                return

            transform.header.frame_id = prefix + '/' + transform.header.frame_id
            transform.child_frame_id = prefix + '/' + transform.child_frame_id

        self.publisher.publish(msg)

        # extra map->odom message in namespace that would have been published by amcl
        mapMsg = TFMessage()
        tf = TransformStamped()
        tf.header.stamp.sec = msg.transforms[0].header.stamp.sec
        tf.header.stamp.nanosec = msg.transforms[0].header.stamp.nanosec
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'odom'

        mapMsg.transforms.append(tf)
        
        self.agent_publishers[prefix].publish(mapMsg)



def main(args=None):
    rclpy.init(args=args)

    config_path = os.path.join(get_package_share_directory('bench_pkg'), 'param/config.yaml')

    with open(config_path, 'r') as stream:
        try:
            config = DotMap(yaml.safe_load(stream))
        except yaml.YAMLError as exc:
            print(exc)
            return

    robots_description_path = config.common.pathToRobotsList

    with open(robots_description_path, 'r') as stream:
        try:
            robots = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            return

    prefixes = []
    for robot in robots:
        prefixes.append(robot['name'])

    tf2_sub = TF2Subscriber(prefixes)

    rclpy.spin(tf2_sub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tf2_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
