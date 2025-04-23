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

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from rclpy.node import Node
import rclpy

import math

class PubSub(Node):
    def __init__(self):
        super().__init__('pubsub')

        # the sauce: http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html
        self.publisher_ = self.create_publisher(Twist, 'chicken_jockey/diff_drive/cmd_vel', 10)
        
        # the sauce: https://docs.ros2.org/foxy/api/sensor_msgs/msg/LaserScan.html
        self.subscriber_ = self.create_subscription(LaserScan, 'chicken_jockey/diff_drive/scan', self.listener_callback, 10)

        self.timer  = self.create_timer(0.5, self.timer_callback)
        self.ranges = []

    def listener_callback(self, msg):
        self.ranges = msg.ranges

    def timer_callback(self):
        msg    = Twist()
        ranges = self.ranges

        # the sauce: https://www.theconstruct.ai/read-laserscan-data/
        front = ranges[0]
        left  = ranges[1]

        self.get_logger().info(str(front) + " is front.")
        self.get_logger().info(str(left) + " is left.")

        # this is what we call a bang bang controller in the biz, ya see?
            # rapidly switching between two states based on sensor data
        # forward --- if no obstacle is in front and obstacle is >= 5.2 to the left ---> rotate left
        if front > 2.0 and left >= 3.0:

            self.get_logger().info("rotating left.")

            msg.linear.x  = 0.0
            msg.angular.z = 0.5

        # forward --- if obstacle is in front or obstacle is <= 2.3 to the left ---> rotate right
        elif front <= 2.0 or left <= 2.0:

            self.get_logger().info("rotating right.")

            msg.linear.x  = 0.0
            msg.angular.z = -0.5

        # start        --- always ---> rotate left
        # rotate left  --- always ---> forward
        # rotate right --- always ---> forward
        else:

            self.get_logger().info("going straight.")

            msg.linear.x  = 5.0
            msg.angular.z = 0.0

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    pubsub = PubSub()
    rclpy.spin(pubsub)

if __name__ == '__main__':
    main()
