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
from consai_game.world_model.ball_activity_model import BallActivityModel, BallState
from consai_game.world_model.game_config_model import GameConfigModel
from consai_game.world_model.referee_model import RefereeModel
from consai_game.utils.geometry import Point

from consai_tools.geometry import geometry_tools as tools

from consai_msgs.msg import MotionCommand

from dataclasses import dataclass, field
from enum import Enum, auto


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


@dataclass
class RobotInfo:
    """単一のロボット情報を保持するデータクラス."""

    # ロボットID
    robot_id: int = 0

    # 目標位置までの距離
    desired_distance: float = float("inf")
    # ボールまでの距離
    ball_distance: float = float("inf")
    # プレースメント位置までの距離
    placement_distance: float = float("inf")

    # 目標位置に到着しているかのフラグ
    arrived: bool = False


@dataclass
class RobotsInfo:
    """自ロボットの情報を保持するデータクラス."""

    robots: dict[int, RobotInfo] = field(default_factory=dict)

    def clear(self):
        """全ロボット情報を初期化して空にするメソッド."""
        self.robots.clear()

    def visible_ids(self) -> list[int]:
        """可視ロボットのIDリストを返すメソッド."""
        return list(self.robots.keys())

    def arrived_ids(self) -> list[int]:
        """目標位置に到達したロボットのIDリストを返すメソッド."""
        return [r.robot_id for r in self.robots.values() if r.arrived]

    def all_arrived(self) -> bool:
        """全ロボットが目標位置に到達しているかを返すメソッド."""
        return all(r.arrived for r in self.robots.values())

    def get(self, robot_id: int) -> RobotInfo:
        """指定したロボットIDのRobotInfoを返す。存在しない場合はKeyErrorメソッド."""
        return self.robots[robot_id]

    def __getitem__(self, robot_id: int) -> RobotInfo:
        """辞書のようにロボットIDでRobotInfoへアクセスできるようにするメソッド."""
        return self.robots[robot_id]

    def __setitem__(self, robot_id: int, value: RobotInfo):
        """辞書のようにロボットIDでRobotInfoを設定できるようにするメソッド."""
        self.robots[robot_id] = value

    def __contains__(self, robot_id: int) -> bool:
        """ロボットIDが含まれているか判定するメソッド."""
        return robot_id in self.robots

    def __len__(self):
        """可視ロボット数を返すメソッド."""
        return len(self.robots)

    def keys(self):
        """可視ロボットのID一覧を返すメソッド."""
        return self.robots.keys()

    def values(self):
        """可視ロボットのRobotInfo一覧を返すメソッド."""
        return self.robots.values()

    def items(self):
        """可視ロボットの(ID, RobotInfo)タプル一覧を返すメソッド."""
        return self.robots.items()


@dataclass
class OurRobotsInfo:
    """自ロボットの情報を保持するデータクラス."""

    our_robots: RobotsInfo = field(default_factory=RobotsInfo)


@dataclass
class TheirRobotsInfo:
    """敵ロボットの情報を保持するデータクラス."""

    their_robots: RobotsInfo = field(default_factory=RobotsInfo)


class ProhibitedKickRobotSearchState(Enum):
    """キック禁止ロボット探索状態を表す列挙型."""

    BEFORE_SEARCH = 0
    SHOULD_FIRST_SEARCH = auto()
    SHOULD_SECOND_SEARCH = auto()
    SEARCH_COMPLETED = auto()


class RobotActivityModel:
    """ロボットの活動状態を保持するクラス."""

    # ロボットが目標位置に到着したと判定する距離[m]
    DIST_ROBOT_TO_DESIRED_THRESHOLD = 0.1
    INVALID_ROBOT_ID = -1

    def __init__(self):
        """RobotActivityModelの初期化."""
        self.our_visible_robots = RobotsInfo()
        self.their_visible_robots = RobotsInfo()
        self.our_ball_receive_score: list[ReceiveScore] = []
        self.our_prohibited_kick_robot_id: int = self.INVALID_ROBOT_ID
        self.prohibited_kick_robot_candidate_id: int = self.INVALID_ROBOT_ID
        self.prohibited_kick_robot_search_state = ProhibitedKickRobotSearchState.BEFORE_SEARCH
        self.number_of_their_robots_in_our_area: int = 0

    def update(
        self,
        robots: RobotsModel,
        ball: BallModel,
        ball_activity: BallActivityModel,
        game_config: GameConfigModel,
        referee: RefereeModel,
    ):
        """ロボットの可視状態を更新し, 距離や状態をセットする関数."""

        # 可視ロボットをRobotInfoで管理
        self.our_visible_robots.clear()
        for robot in robots.our_robots.values():
            if robot.is_visible:
                info = RobotInfo(
                    robot_id=robot.robot_id,
                    ball_distance=tools.get_distance(robot.pos, ball.pos),
                    placement_distance=tools.get_distance(robot.pos, referee.placement_pos),
                )
                self.our_visible_robots[robot.robot_id] = info

        self.their_visible_robots.clear()
        for robot in robots.their_robots.values():
            if robot.is_visible:
                info = RobotInfo(
                    robot_id=robot.robot_id,
                    ball_distance=tools.get_distance(robot.pos, ball.pos),
                    placement_distance=tools.get_distance(robot.pos, referee.placement_pos),
                )
                self.their_visible_robots[robot.robot_id] = info

        # ボールに近い順
        self.our_robots_by_ball_distance = [
            r.robot_id for r in sorted(self.our_visible_robots.values(), key=lambda x: x.ball_distance)
        ]
        self.their_robots_by_ball_distance = [
            r.robot_id for r in sorted(self.their_visible_robots.values(), key=lambda x: x.ball_distance)
        ]
        # プレースメント位置に近い順
        self.our_robots_by_placement_distance = [
            r.robot_id for r in sorted(self.our_visible_robots.values(), key=lambda x: x.placement_distance)
        ]

        # ボール受け取りスコア
        self.our_ball_receive_score = self.calc_ball_receive_score_list(
            robots={rid: robots.our_robots[rid] for rid in self.our_visible_robots.visible_ids()},
            ball=ball,
            ball_activity=ball_activity,
            game_config=game_config,
        )

        # 自陣にいる相手ロボット数
        self.number_of_their_robots_in_our_area = sum(
            1 for r in self.their_visible_robots.values() if robots.their_robots[r.robot_id].pos.x < 0
        )

        self.update_prohibited_kick_robot(ball_activity, referee)

    def update_our_robots_arrived(self, robots: dict[int, Robot], commands: list[MotionCommand]):
        """各ロボットが目標位置に到達したかをRobotInfoにセット"""
        for command in commands:
            if command.robot_id not in robots:
                continue
            robot = robots[command.robot_id]
            dist = tools.get_distance(robot.pos, command.desired_pose)
            if command.robot_id in self.our_visible_robots:
                self.our_visible_robots[command.robot_id].arrived = dist < self.DIST_ROBOT_TO_DESIRED_THRESHOLD

    @property
    def our_robots_arrived(self) -> bool:
        """すべての自ロボットが目標位置に到達したか"""
        return self.our_visible_robots.all_arrived()

    def our_robot_arrived(self, robot_id: int) -> bool:
        """指定したロボットが目標位置に到達したか"""
        if robot_id in self.our_visible_robots:
            return self.our_visible_robots[robot_id].arrived
        return False

    def count_their_robots(self, robots: dict[int, Robot]) -> int:
        """自陣にいる相手ロボットの数を返す関数."""
        counter = 0

        for robot in robots.values():
            if robot.pos.x < 0:
                counter += 1

        return counter

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

    def update_prohibited_kick_robot(self, ball_activity: BallActivityModel, referee: RefereeModel):
        """レフェリー信号を見て、ダブルタッチをしてはいけないロボットを更新する関数."""
        # ストップゲームで初期化する
        if referee.stop:
            self.prohibited_kick_robot_search_state = ProhibitedKickRobotSearchState.BEFORE_SEARCH
            self.our_prohibited_kick_robot_id = self.INVALID_ROBOT_ID
            return

        # ボール保持者を候補としてセットする
        if ball_activity.ball_holder:
            if ball_activity.ball_holder.is_our_team:
                self.prohibited_kick_robot_candidate_id = ball_activity.ball_holder.robot.robot_id

        if self.prohibited_kick_robot_search_state == ProhibitedKickRobotSearchState.BEFORE_SEARCH:
            # 探索開始前

            # 自チームのセットプレイになったら探索を開始する
            if referee.our_free_kick or referee.our_kick_off_start:
                self.prohibited_kick_robot_search_state = ProhibitedKickRobotSearchState.SHOULD_FIRST_SEARCH

        elif self.prohibited_kick_robot_search_state == ProhibitedKickRobotSearchState.SHOULD_FIRST_SEARCH:
            # 一番はじめにボールをけるロボットを探す

            if ball_activity.ball_state == BallState.OURS_KICKED:
                # 自チームがボールを蹴ったらIDをセットする
                self.our_prohibited_kick_robot_id = self.prohibited_kick_robot_candidate_id
                self.prohibited_kick_robot_search_state = ProhibitedKickRobotSearchState.SHOULD_SECOND_SEARCH
            elif ball_activity.ball_state == BallState.THEIRS_KICKED:
                # 相手がボールを蹴ったら探索終了
                self.prohibited_kick_robot_search_state = ProhibitedKickRobotSearchState.SEARCH_COMPLETED

        elif self.prohibited_kick_robot_search_state == ProhibitedKickRobotSearchState.SHOULD_SECOND_SEARCH:
            # 二回目にボールに触ったロボット探す
            if ball_activity.ball_state == BallState.THEIRS_KICKED:
                # 相手がボールを蹴ったら探索終了
                self.prohibited_kick_robot_search_state = ProhibitedKickRobotSearchState.SEARCH_COMPLETED
                self.our_prohibited_kick_robot_id = self.INVALID_ROBOT_ID
            elif ball_activity.ball_state == BallState.OURS_KICKED:
                if self.our_prohibited_kick_robot_id != self.prohibited_kick_robot_candidate_id:
                    # 違うロボットがボールを蹴ったら探索終了
                    self.prohibited_kick_robot_search_state = ProhibitedKickRobotSearchState.SEARCH_COMPLETED
                    self.our_prohibited_kick_robot_id = self.INVALID_ROBOT_ID
