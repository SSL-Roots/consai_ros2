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

"""可視ロボットの識別と順序づけを行うモデル."""

from consai_game.world_model.robots_model import Robot, RobotsModel
from consai_game.world_model.ball_model import BallModel
from consai_game.world_model.ball_activity_model import BallActivityModel
from consai_game.world_model.game_config_model import GameConfigModel
from consai_game.world_model.referee_model import RefereeModel
from consai_game.utils.geometry import Point

from consai_tools.geometry import geometry_tools as tools

from consai_msgs.msg import MotionCommand

from dataclasses import dataclass


@dataclass
class ReceiveScore:
    """ボールをどれだけ受け取りやすいかを保持するデータクラス."""

    robot_id: int = 0
    intercept_time: float = float("inf")  # あと何秒後にボールを受け取れるか


@dataclass
class OurRobotsArrived:
    """自ロボットが目標位置に到達したか保持するデータクラス."""

    robot_id: int = 0
    arrived: bool = False


class RobotActivityModel:
    """ロボットの活動状態を保持するクラス."""

    # ロボットが目標位置に到着したと判定する距離[m]
    DIST_ROBOT_TO_DESIRED_THRESHOLD = 0.1

    def __init__(self):
        """ロボットの可視状態と順序リストの初期化関数."""
        self.ordered_our_visible_robots: list[int] = []
        self.ordered_their_visible_robots: list[int] = []
        self.our_visible_robots: list[int] = []
        self.their_visible_robots: list[int] = []
        self.our_robots_by_ball_distance: list[int] = []
        self.their_robots_by_ball_distance: list[int] = []
        self.our_robots_by_placement_distance: list[int] = []
        self.our_ball_receive_score: list[ReceiveScore] = []
        self.our_robots_arrived_list: list[OurRobotsArrived] = []

    def update(
        self,
        robots: RobotsModel,
        ball: BallModel,
        ball_activity: BallActivityModel,
        game_config: GameConfigModel,
        referee: RefereeModel,
    ):
        """ロボットの可視状態を更新し, 順序づけされたIDリストを更新する関数."""
        self.our_visible_robots = [robot.robot_id for robot in robots.our_robots.values() if robot.is_visible]
        self.their_visible_robots = [robot.robot_id for robot in robots.their_robots.values() if robot.is_visible]

        self.ordered_our_visible_robots = self.ordered_merge(
            self.ordered_our_visible_robots,
            self.our_visible_robots,
        )
        self.ordered_their_visible_robots = self.ordered_merge(
            self.ordered_their_visible_robots,
            self.their_visible_robots,
        )

        # ボールに近い順にリストを作る
        self.our_robots_by_ball_distance = [
            robot_id
            for robot_id, _ in self.robot_ball_distances(
                robots.our_visible_robots,
                ball,
            )
        ]
        self.their_robots_by_ball_distance = [
            robot_id
            for robot_id, _ in self.robot_ball_distances(
                robots.their_visible_robots,
                ball,
            )
        ]

        # プレースメント位置に近い順にリストを作る
        self.our_robots_by_placement_distance = [
            robot_id
            for robot_id, _ in self.robot_placement_distances(
                robots.our_visible_robots,
                referee,
            )
        ]

        # ボールを受け取れるスコアを計算する
        self.our_ball_receive_score = self.calc_ball_receive_score_list(
            robots=robots.our_visible_robots,
            ball=ball,
            ball_activity=ball_activity,
            game_config=game_config,
        )

    def ordered_merge(self, prev_list: list[int], new_list: list[int]) -> list[int]:
        """過去の順序を保ちながら, 新しいリストでマージする関数."""
        # new_listに存在するものを残す
        output_list = [r for r in prev_list if r in new_list]

        # 新しい要素を追加する
        output_list.extend([r for r in new_list if r not in output_list])

        return output_list

    def robot_something_distances(self, robots: dict[int, Robot], target: Point) -> list[tuple[int, float]]:
        """ロボットとなにかの距離を計算し, 距離の昇順でソートしたリストを返す関数."""

        distances = []

        for robot in robots.values():
            distance = tools.get_distance(robot.pos, target)
            distances.append((robot.robot_id, distance))

        # 距離でソート
        distances.sort(key=lambda x: x[1])

        return distances

    def robot_ball_distances(self, robots: dict[int, Robot], ball: BallModel) -> list[tuple[int, float]]:
        """ロボットとボールの距離を計算し, 距離の昇順でソートしたリストを返す関数."""
        target = Point(x=ball.pos.x, y=ball.pos.y)

        return self.robot_something_distances(robots, target)

    def robot_placement_distances(self, robots: dict[int, Robot], referee: RefereeModel) -> list[tuple[int, float]]:
        """ロボットとプレースメント位置の距離を計算し, 距離の昇順でソートしたリストを返す関数."""
        return self.robot_something_distances(robots, referee.placement_pos)

    def calc_ball_receive_score_list(
        self, robots: dict[int, Robot], ball: BallModel, ball_activity: BallActivityModel, game_config: GameConfigModel
    ) -> list[ReceiveScore]:
        """ロボットごとにボールを受け取れるスコアを計算する."""

        # ボールが動いていない場合は、スコアをデフォルト値にする
        if not ball_activity.ball_is_moving:
            return [ReceiveScore(robot_id=robot.robot_id) for robot in robots.values()]

        score_list = []
        for robot in robots.values():
            score_list.append(
                ReceiveScore(
                    robot_id=robot.robot_id,
                    intercept_time=self.calc_intercept_time(robot, ball, game_config),
                )
            )

        # intercept_timeが小さい順にソート
        score_list.sort(key=lambda x: x.intercept_time)
        return score_list

    def calc_intercept_time(self, robot: Robot, ball: BallModel, game_config: GameConfigModel) -> float:
        """ロボットがボールを受け取るまでの時間を計算する関数."""

        # ボールを中心に、ボールの速度方向を+x軸にした座標系を作る
        trans = tools.Trans(ball.pos, tools.get_vel_angle(ball.vel))

        # ロボットの位置を変換
        tr_robot_pos = trans.transform(robot.pos)

        # TODO: ボールを後ろから追いかけて受け取れるようになったら、計算を変更する
        if tr_robot_pos.x < 0:
            return float("inf")

        # ロボットからボール軌道まで垂線を引き、
        # その交点にボールが到達するまでの時間を計算する
        ball_arrival_distance = tr_robot_pos.x
        intercept_time = ball_arrival_distance / tools.get_norm(ball.vel)

        # ボールが到達するまでの時間で、ロボットがどれだけ移動できるかを計算する
        # TODO: ロボットの現在速度、加速度を考慮すべき
        available_distance = intercept_time * game_config.robot_max_linear_vel

        # ボール軌道からロボットまでの距離
        robot_arrival_distance = abs(tr_robot_pos.y)

        # ボールが到着するまでにロボットが移動できれば、intercept_timeを返す
        if available_distance >= robot_arrival_distance:
            return intercept_time
        return float("inf")

    def update_our_robots_arrived(self, our_visible_robots: dict[int, Robot], commands: list[MotionCommand]) -> bool:
        """各ロボットが目標位置に到達したか判定する関数."""

        # 初期化
        self.our_robots_arrived_list = []
        # エラー処理
        if len(our_visible_robots) == 0 or len(commands) == 0:
            return

        # 更新
        for command in commands:
            if command.robot_id not in our_visible_robots.keys():
                continue

            robot = our_visible_robots[command.robot_id]
            robot_pos = robot.pos
            desired_pose = command.desired_pose
            # ロボットと目標位置の距離を計算
            dist_robot_to_desired = tools.get_distance(robot_pos, desired_pose)
            # 目標位置に到達したか判定結果をリストに追加
            self.our_robots_arrived_list.append(
                OurRobotsArrived(
                    robot_id=robot.robot_id,
                    arrived=dist_robot_to_desired < self.DIST_ROBOT_TO_DESIRED_THRESHOLD,
                )
            )

    @property
    def our_robots_arrived(self) -> bool:
        """すべての自ロボットが目標位置に到達したかを返す関数."""
        return all([robot.arrived for robot in self.our_robots_arrived_list])

    def our_robot_arrived(self, robot_id: int) -> bool:
        """指定したロボットが目標位置に到達したかを返す関数."""
        for robot in self.our_robots_arrived_list:
            if robot.robot_id == robot_id:
                return robot.arrived
        return False
