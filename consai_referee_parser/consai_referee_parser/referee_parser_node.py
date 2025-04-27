# Copyright 2025 Roots
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


"""
referee_parser_node モジュール.

このモジュールは ROS2 ノードとして動作する.
Referee メッセージを受け取り, consai_robot_controllerが使用できる形式でpublishする.
"""

import threading
from rclpy.node import Node
from robocup_ssl_msgs.msg import Referee


class RefereeParserNode(Node):
    """ """

    def __init__(self, team_is_yellow: bool = False, invert: bool = False):
        """
        初期化.
        """
        super().__init__("referee_parser_node")
        self.lock = threading.Lock()

        self.team_is_yellow = team_is_yellow
        self.invert = invert

        self.sub_referee = self.create_subscription(Referee, "referee", self.callback_referee, 10)

    def callback_referee(self, msg: Referee):
        self.get_logger().info("referee_parser_node: referee msg received")
