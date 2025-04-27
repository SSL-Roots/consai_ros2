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

from consai_msgs.msg import State2D
from consai_msgs.msg import RefereeSupportInfo
from consai_visualizer_msgs.msg import Objects
from consai_referee_parser import referee_to_vis_msg
from rclpy import qos
from rclpy.node import Node
from robocup_ssl_msgs.msg import Referee
import threading


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

        self.pub_visualizer_objects = self.create_publisher(Objects, "visualizer_objects", qos.qos_profile_sensor_data)

        self.sub_referee = self.create_subscription(Referee, "referee", self.callback_referee, 10)
        self.sub_referee_support_info = self.create_subscription(
            RefereeSupportInfo, "parsed_referee/referee_support_info", self.callback_referee_support_info, 10
        )
        self.support_info = RefereeSupportInfo()

    def callback_referee(self, msg: Referee):
        with self.lock:
            self.pub_designated_position.publish(self.support_info.placement_pos)

            self.pub_visualizer_objects.publish(
                referee_to_vis_msg.vis_info(
                    referee=msg,
                    blue_bots=self.support_info.blue_robot_num,
                    yellow_bots=self.support_info.yellow_robot_num,
                    placement_pos=self.support_info.placement_pos,
                )
            )

    def callback_referee_support_info(self, msg: RefereeSupportInfo):
        with self.lock:
            self.support_info = msg
