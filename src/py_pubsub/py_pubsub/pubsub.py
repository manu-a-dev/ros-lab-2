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

        # variables for the low-pass filter:
        self.filtered_front = 0.0
        self.filtered_left  = 0.0
        
        # equation came from the slides.
            # 0.2 is the fc.
            # 0.5 is the dt.
        self.beta = (2 * math.pi * 0.4 * 0.5) / (2 * math.pi * 0.5 * 0.4 + 1)

    def listener_callback(self, msg):
        self.ranges = msg.ranges

    def timer_callback(self):
        msg    = Twist()
        ranges = self.ranges

        if len(ranges) < 2:
            self.get_logger().info("start the simulation, duh!")
            return

        # the sauce: https://www.theconstruct.ai/read-laserscan-data/
        previous_front = ranges[0]
        previous_left  = ranges[1]

        # apply the low-pass filter, kronk!
        self.filtered_front = self.beta * previous_front + (1 - self.beta) * self.filtered_front
        self.filtered_left  = self.beta * previous_left + (1 - self.beta) * self.filtered_left

        front = self.filtered_front
        left  = self.filtered_left

        # for section 2:
        # front = ranges[0]
        # left  = ranges[1]

        # if no obstacle is in front and obstacle is >= 2.0 to the left . . .
        if front > 2.0 and left >= 2.0:
            self.get_logger().info("rotating left.")
            msg.linear.x  = 0.5 # causes the swaying.
            msg.angular.z = 0.25
    
        # if obstacle is in front . . .
        elif front <= 1.8:
            self.get_logger().info("backing up.")
            msg.linear.x  = -10.0 # these are capped in model.sdf, so no point in having them this high.
            msg.angular.z = -10.0

        # if obstacle is <= 1.5 to the left . . .
        elif left <= 1.5:
            self.get_logger().info("rotating right.")
            msg.linear.x  = 0.5 # causes the swaying.
            msg.angular.z = -0.25

        else:
            self.get_logger().info("going straight.")
            msg.linear.x = 5.0

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    pubsub = PubSub()
    rclpy.spin(pubsub)

if __name__ == '__main__':
    main()
