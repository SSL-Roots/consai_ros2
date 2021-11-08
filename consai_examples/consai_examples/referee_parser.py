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

from rclpy.node import Node
from robocup_ssl_msgs.msg import Referee


# refereeトピックを解読するノード
class RefereeParser(Node):
    def __init__(self, our_team_is_yellow=False):
        super().__init__('operator')

        self._our_team_is_yellow = our_team_is_yellow
        self._sub_referee = self.create_subscription(
            Referee, 'referee', self._referee_callback, 10)

    def _referee_callback(self, msg):
        self.get_logger().info("received!")
