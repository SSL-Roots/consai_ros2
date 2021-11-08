#!/usr/bin/env python3
# coding: UTF-8

# Copyright 2021 Roots
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

from robot_operator import RobotOperator
from referee_parser import RefereeParser
import math
import rclpy
from rclpy.executors import SingleThreadedExecutor


def main():
    # 実行したい関数のコメントを外してください
    while rclpy.ok():
        executor.spin_once(1)

if __name__ == '__main__':
    rclpy.init(args=None)

    our_team_is_yellow = False
    operator_node = RobotOperator(our_team_is_yellow)
    referee_parser = RefereeParser(our_team_is_yellow)


    executor = SingleThreadedExecutor()
    executor.add_node(operator_node)
    executor.add_node(referee_parser)

    try:
        main()
    except KeyboardInterrupt:
        pass

    executor.shutdown()
    rclpy.shutdown()
