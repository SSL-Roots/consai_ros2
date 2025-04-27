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
Referee メッセージを受け取り, designated_positionをpublishする.
"""

import threading
from rclpy.node import Node
from robocup_ssl_msgs.msg import Referee
from consai_msgs.msg import State2D


class RefereeParserNode(Node):
    def __init__(self, team_is_yellow: bool = False, invert: bool = False):
        """
        初期化.

        Args:
            team_is_yellow (bool): 自チームがイエローチームかどうか
            invert (bool): フィールドサイドを反転させるかどうか
        """
        super().__init__("referee_parser_node")
        self.lock = threading.Lock()

        self.team_is_yellow = team_is_yellow
        self.invert = invert

        self.pub_designated_position = self.create_publisher(State2D, "parsed_referee/designated_position", 10)
        self.sub_referee = self.create_subscription(Referee, "referee", self.callback_referee, 10)

    def callback_referee(self, msg: Referee):
        if len(msg.designated_position) > 0:
            position = State2D()
            position.x = msg.designated_position[0].x * 0.001  # mm to meters
            position.y = msg.designated_position[0].y * 0.001  # mm to meters

            # フィールドサイドを反転しているときは、目標座標も反転させる
            if self.invert:
                position.x *= -1.0
                position.y *= -1.0

            self.pub_designated_position.publish(position)
