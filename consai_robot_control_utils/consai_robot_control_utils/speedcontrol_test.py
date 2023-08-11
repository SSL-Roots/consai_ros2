#!/usr/bin/env python3
# coding: UTF-8

# Copyright 2023 Roots
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

import csv
import rclpy
from rclpy.node import Node

from consai_frootspi_msgs.msg import RobotCommand


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('speedcontrol_test')

        # parameters
        self.declare_parameter('csv_path', '')
        self.csv_path = self.get_parameter('csv_path').get_parameter_value().string_value
        self.declare_parameter('robot_id', 0)
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().integer_value
        self.declare_parameter('loop', False)
        self.loop = self.get_parameter('loop').get_parameter_value().bool_value
        self.declare_parameter('yellow', False)
        self.yellow = self.get_parameter('yellow').get_parameter_value().bool_value

        # publishers
        self.publisher_ = self.create_publisher(RobotCommand, f"/robot{self.robot_id}/command", 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.profile = self.load_profile(self.csv_path)
        self.latest_pub_time = self.get_clock().now()

        self.read_index = 0

    def timer_callback(self):

        msg = RobotCommand()
        msg.robot_id = self.robot_id
        msg.team_is_yellow = self.yellow

        print(self.profile[self.read_index])

        msg.velocity_x = float(self.profile[self.read_index][1])
        msg.velocity_y = float(self.profile[self.read_index][2])
        msg.velocity_theta = float(self.profile[self.read_index][3])

        msg.kick_power = 0.0
        msg.chip_kick = False
        msg.dribble_power = 0.0

        self.publisher_.publish(msg)

        now_time = self.get_clock().now()
        # dt = self.latest_pub_time - now_time

        self.latest_pub_time = now_time

        self.read_index += 1

        # self.profileが末尾まで達したらプログラム終了
        if self.read_index >= len(self.profile):
            if self.loop:
                self.read_index = 0
            else:
                self.get_logger().info('finished')
                self.destroy_node()
                rclpy.shutdown()

    def load_profile(self, csv_path):
        """
        CSVファイルから移動プロファイルを読み込む.

        Args:
            csv_path (str): CSVファイルのパス

        Returns
        -------
            list: 移動プロファイル

        """
        with open(csv_path) as f:
            reader = csv.reader(f)
            profile = [row for row in reader]

        # ヘッダーを削除
        profile.pop(0)

        return profile


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
