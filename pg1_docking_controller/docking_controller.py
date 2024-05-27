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

import rclpy

import time

from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point




class DockingController(Node):

    def __init__(self):
        super().__init__('docking_controller')
        self.subscription = self.create_subscription(Point, 'detected_ball', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel_docking', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.rcv_timeout_secs = 1
        self.angular_chase_multiplier = 0.1
        self.forward_chase_speed = 0.1
        self.search_angular_speed = 0.5
        self.max_size_thresh = 0.05
        self.filter_value = 0.9

        self.swing_counter = 0
        self.backtime_time = 2.5
        self.is_backing_up = 0
        self.at_target = 0
        self.in_position = 0



        self.target_val = 0.0
        self.target_dist = 0.0
        self.lastrcvtime = time.time() - 10000
        self.backtimestart = time.time() - 10000

    def timer_callback(self):
        msg = Twist()
        if(not self.at_target):
            if (time.time() - self.lastrcvtime < self.rcv_timeout_secs or self.is_backing_up):
                

                if (self.target_dist < self.max_size_thresh and not self.in_position):
                    self.backtimestart = time.time()
                    self.get_logger().info('Target: {}'.format(self.target_val))

                    msg.linear.x = -self.forward_chase_speed
                    msg.angular.z = -self.angular_chase_multiplier*self.target_val
                    
                else:

                    self.in_position = 1
                    msg.angular.z = -self.angular_chase_multiplier*self.target_val

                    if(time.time() - self.backtimestart < self.backtime_time and (abs(self.target_val) < 0.1 or self.is_backing_up)):
                        msg.linear.x = -self.forward_chase_speed
                        self.is_backing_up = 1
                        self.get_logger().info('Backing up')
                    elif (time.time() - self.backtimestart < self.backtime_time and abs(self.target_val) >= 0.1):
                        self.backtimestart = time.time()
                        msg.angular.z = -self.angular_chase_multiplier*self.target_val
                        self.get_logger().info('aligning')
                    else:
                        self.at_target = 1

            else:
                self.get_logger().info('Target lost')


                if(self.swing_counter < 20):
                    msg.angular.z = self.search_angular_speed
                elif(self.swing_counter < 40):
                    msg.angular.z = -self.search_angular_speed
                elif(self.swing_counter >= 40):
                    self.swing_counter = 0

                self.swing_counter += 1
        else:
            print('arrived at target')


        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        f = self.filter_value
        self.target_val = self.target_val * f + msg.x * (1-f)
        self.target_dist = self.target_dist * f + msg.z * (1-f)
        self.lastrcvtime = time.time()
        # self.get_logger().info('Received: {} {}'.format(msg.x, msg.y))



def main(args=None):
    rclpy.init(args=args)

    docking_controller = DockingController()

    rclpy.spin(docking_controller)

    docking_controller.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()