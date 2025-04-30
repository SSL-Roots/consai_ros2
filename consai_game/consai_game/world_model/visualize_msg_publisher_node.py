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
WorldModelの情報を可視化トピックとしてpublishするノードの定義.

キックターゲットやボールの状態をObjectsメッセージとしてGUIに送信する.
"""

from rclpy import qos
from rclpy.node import Node

from consai_visualizer_msgs.msg import Objects, ShapeCircle, ShapeLine, ShapeText

from consai_game.world_model.ball_model import BallModel
from consai_game.world_model.ball_activity_model import BallActivityModel, BallState
from consai_game.world_model.robot_activity_model import RobotActivityModel
from consai_game.world_model.kick_target_model import KickTargetModel
from consai_game.world_model.robots_model import RobotsModel
from consai_game.world_model.world_model import WorldModel


class VisualizeMsgPublisherNode(Node):
    """WorldModelをGUIに描画するためのトピックをpublishするノード."""

    def __init__(self):
        """ノードの初期化関数."""
        super().__init__("vis_msg_publisher_node")
        self.pub_visualizer_objects = self.create_publisher(Objects, "visualizer_objects", qos.qos_profile_sensor_data)

    def publish(self, world_model: WorldModel):
        """WorldModelをGUIに描画するためのトピックをpublishする."""
        self.pub_visualizer_objects.publish(
            self.kick_target_to_vis_msg(kick_target=world_model.kick_target, ball=world_model.ball)
        )

        self.pub_visualizer_objects.publish(
            self.ball_activity_to_vis_msg(activity=world_model.ball_activity, ball=world_model.ball)
        )

        self.pub_visualizer_objects.publish(
            self.threats_to_vis_msg(threats=world_model.threats, robots=world_model.robots)
        )
        self.pub_visualizer_objects.publish(
            self.robot_activity_to_vis_msg(robot_activity=world_model.robot_activity, robots=world_model.robots)
        )

    def kick_target_to_vis_msg(self, kick_target: KickTargetModel, ball: BallModel) -> Objects:
        """kick_targetをObjectsメッセージに変換する."""
        vis_obj = Objects()
        vis_obj.layer = "game"
        vis_obj.sub_layer = "kick_target"
        vis_obj.z_order = 4

        # ボールとシュートターゲットを結ぶ直線を引く
        for i, target in enumerate(kick_target.shoot_target_list):
            line = ShapeLine()
            line.p1.x = ball.pos.x
            line.p1.y = ball.pos.y
            line.p2.x = target.pos.x
            line.p2.y = target.pos.y

            line.size = 1
            line.color.name = "black"
            # ベストシュートターゲットの色を赤にする
            if i == 0:
                line.color.name = "red"

            # シュート成功率が高いほど色を刻する
            line.color.alpha = min(1.0, target.success_rate / 100.0)
            line.caption = f"rate: {target.success_rate}"

            vis_obj.lines.append(line)

        # ボールとパスターゲットを結ぶ直線を引く
        for i, target in enumerate(kick_target.pass_target_list):
            line = ShapeLine()
            line.p1.x = ball.pos.x
            line.p1.y = ball.pos.y
            line.p2.x = target.robot_pos.x
            line.p2.y = target.robot_pos.y

            line.size = 1
            line.color.name = "blue"
            # ベストパスターゲットの色を変える
            if i == 0:
                line.color.name = "yellow"
                line.size = 2

            # シュート成功率が高いほど色を刻する
            line.color.alpha = min(1.0, target.success_rate / 100.0)
            line.caption = f"rate: {target.success_rate}"

            vis_obj.lines.append(line)

        return vis_obj

    def ball_activity_to_vis_msg(self, activity: BallActivityModel, ball: BallModel) -> Objects:
        """ball_activityをObjectsメッセージに変換する."""
        OUR_COLOR = "floralwhite"
        THEIR_COLOR = "gray"

        vis_obj = Objects()
        vis_obj.layer = "game"
        vis_obj.sub_layer = "ball_activity"
        vis_obj.z_order = 4

        # ボール付近にstate文字列を表示する
        state_text = ShapeText()
        state_text.text = activity.ball_state.name
        state_text.x = ball.pos.x + 0.1
        state_text.y = ball.pos.y + 0.1
        state_text.size = 10
        state_text.color.name = "white"
        vis_obj.texts.append(state_text)

        # ball_stateに合わせて、ボールの裏に円を描く
        state_circle = ShapeCircle()
        state_circle.center.x = ball.pos.x
        state_circle.center.y = ball.pos.y
        state_circle.radius = 0.1
        state_circle.line_size = 1

        if activity.ball_state in [BallState.OURS, BallState.OURS_KICKED]:
            state_circle.line_color.name = OUR_COLOR
            state_circle.fill_color.name = OUR_COLOR
            vis_obj.circles.append(state_circle)
        elif activity.ball_state in [BallState.THEIRS, BallState.THEIRS_KICKED]:
            state_circle.line_color.name = THEIR_COLOR
            state_circle.fill_color.name = THEIR_COLOR
            vis_obj.circles.append(state_circle)

        # ボールのストップ位置を描画
        if activity.ball_is_moving:
            stop_pos_circle = ShapeCircle()
            stop_pos_circle.center.x = activity.ball_stop_position.x
            stop_pos_circle.center.y = activity.ball_stop_position.y
            stop_pos_circle.radius = 0.2
            stop_pos_circle.line_size = 2
            stop_pos_circle.line_color.name = "coral"
            stop_pos_circle.fill_color.name = "coral"
            stop_pos_circle.fill_color.alpha = 0.0
            stop_pos_circle.caption = "stop_pos"
            # ボールがゴールに入る場合は円を塗りつぶす
            if activity.ball_will_enter_their_goal:
                stop_pos_circle.fill_color.alpha = 1.0
                stop_pos_circle.caption = "will enter goal"

            vis_obj.circles.append(stop_pos_circle)

        return vis_obj

    def threats_to_vis_msg(self, threats, robots) -> Objects:
        """threatsをObjectsメッセージに変換する."""
        vis_obj = Objects()
        vis_obj.layer = "game"
        vis_obj.sub_layer = "threats"
        vis_obj.z_order = 4

        # 各ロボットの上に驚異度を表示
        for i, threat in enumerate(threats.threats):
            robot = robots.their_robots[threat.robot_id]
            text = ShapeText()
            text.text = f"{i+1}: [{threat.score}]"
            text.x = robot.pos.x - 0.2
            text.y = robot.pos.y + 0.2  # ロボットの上に表示
            text.size = 12
            text.color.name = "red"
            vis_obj.texts.append(text)

        return vis_obj

    def robot_activity_to_vis_msg(self, robot_activity: RobotActivityModel, robots: RobotsModel) -> Objects:
        """robot_activityをObjectsメッセージに変換する."""
        vis_obj = Objects()
        vis_obj.layer = "game"
        vis_obj.sub_layer = "robot_activity"
        vis_obj.z_order = 4

        # ボールの受け取りランキングを描画
        COLOR_BEST_RECEIVE = "tomato"
        COLOR_RECEIVE = "gray"
        for i, score in enumerate(robot_activity.our_ball_receive_score):
            if score.intercept_time == float("inf"):
                return vis_obj

            # ロボットの周りに円を描いて、レシーブ可能なことを描画する
            robot_pos = robots.our_visible_robots[score.robot_id].pos

            circle = ShapeCircle()
            circle.center.x = robot_pos.x
            circle.center.y = robot_pos.y
            circle.radius = 0.3
            circle.line_size = 1
            circle.line_color.name = COLOR_RECEIVE
            circle.fill_color.name = COLOR_RECEIVE
            circle.caption = f"{i+1}: {score.intercept_time:.2f}"
            if i == 0:
                circle.line_color.name = COLOR_BEST_RECEIVE
                circle.fill_color.name = COLOR_BEST_RECEIVE

            vis_obj.circles.append(circle)

        return vis_obj
