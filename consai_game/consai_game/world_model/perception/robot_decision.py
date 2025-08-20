import numpy as np

from consai_game.utils.geometry import Point
from consai_game.world_model.ball_model import BallModel
from consai_game.world_model.robots_model import Robot

from consai_msgs.msg import State2D

from consai_tools.geometry import geometry_tools as tools

from consai_msgs.msg import MotionCommand

from dataclasses import dataclass, field


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


class RobotDecision:
    """ロボットやボールの位置関係を判定するクラス."""

    ANGLE_BALL_TO_ROBOT_THRESHOLD = 120  # ボールが後方に居るとみなす角度[degree]
    MINIMAL_THETA_THRESHOLD = 45  # 最低限満たすべきロボットの角度
    WIDTH_THRESHOLD = 0.03  # 直線に乗っているかの距離
    DIST_ROBOT_TO_DESIRED_THRESHOLD = 0.1  # ロボットが目標位置に到着したと判定する距離[m]

    def obstacle_exists(ball: BallModel, robots: dict[int, Robot], target: State2D, tolerance) -> bool:
        """ターゲット位置に障害物（ロボット）が存在するかを判定する関数."""

        for robot in robots.values():
            if tools.is_on_line(pose=robot.pos, line_pose1=ball.pos, line_pose2=target, tolerance=tolerance):
                return True
        return False

    def is_robot_inside_pass_area(ball: BallModel, robot: Robot) -> bool:
        """味方ロボットがパスを出すロボットとハーフライン両サイドを結んでできる五角形のエリア内にいるかを判別する関数"""

        _half_width = 4.5

        if robot.pos.x < 0.0:
            return False

        upper_side_slope, upper_side_intercept, flag = tools.get_line_parameter(ball.pos, Point(0.0, _half_width))
        lower_side_slope, lower_side_intercept, flag = tools.get_line_parameter(ball.pos, Point(0.0, _half_width))

        if upper_side_slope is None or lower_side_slope is None:
            if ball.pos.x > robot.pos.x:
                return False
        else:
            upper_y_on_line = upper_side_intercept + upper_side_slope * robot.pos.x
            lower_y_on_line = lower_side_intercept + lower_side_slope * robot.pos.x
            if robot.pos.y < upper_y_on_line and robot.pos.y < lower_y_on_line:
                return False
        return True

    def is_robot_on_kick_line(
        robot_pos: State2D, ball_pos: State2D, target_pos: State2D, width_threshold: float
    ) -> bool:
        """ボールからターゲットまでの直線上にロボットが居るかを判定するメソッド.

        ターゲットまでの距離が遠いと、角度だけで狙いを定めるのは難しいため、位置を使って判定する.
        """

        minimal_theta_threshold = 45  # 最低限満たすべきロボットの角度

        # ボールからターゲットへの座標系を作成
        trans = tools.Trans(ball_pos, tools.get_angle(ball_pos, target_pos))
        tr_robot_pos = trans.transform(robot_pos)
        tr_robot_theta = trans.transform_angle(robot_pos.theta)

        # ボールより前にロボットが居る場合
        if tr_robot_pos.x > 0.0:
            return False

        # ターゲットを向いていない
        if abs(tr_robot_theta) > np.deg2rad(minimal_theta_threshold):
            return False

        if abs(tr_robot_pos.y) > width_threshold:
            return False

        return True

    def update_our_robots_arrived(
        self, robots: dict[int, Robot], commands: list[MotionCommand], our_visible_robots: RobotInfo
    ) -> bool:
        """各ロボットが目標位置に到達したかをRobotInfoにセット"""
        for command in commands:
            if command.robot_id not in robots:
                continue
            robot = robots[command.robot_id]
            dist = tools.get_distance(robot.pos, command.desired_pose)
            if command.robot_id in our_visible_robots:
                our_visible_robots[command.robot_id].arrived = dist < self.DIST_ROBOT_TO_DESIRED_THRESHOLD

    # ball_approach.py
    def robot_is_backside(self, robot_pos: State2D, ball_pos: State2D, ball_stop_pos: State2D) -> bool:
        """ボールからターゲットを見て、ロボットが後側に居るかを判定する."""
        # ボール最終目標地点からボールへの座標系を作成
        trans = tools.Trans(ball_stop_pos, tools.get_angle(ball_stop_pos, ball_pos))
        tr_robot_pos = trans.transform(robot_pos)

        # ボールから見たロボットの位置の角度
        # ボールの後方にいれば角度は90度以上
        tr_ball_to_robot_angle = tools.get_angle(State2D(x=0.0, y=0.0), tr_robot_pos)

        if abs(tr_ball_to_robot_angle) > np.deg2rad(self.ANGLE_BALL_TO_ROBOT_THRESHOLD):
            return True
        return False

    def robot_is_on_receiving_line(self, robot_pos: State2D, ball_pos: State2D, ball_stop_pos: State2D) -> bool:
        """ボールからターゲットまでの直線上にロボットが居るかを判定する.

        ターゲットまでの距離が遠いと、角度だけで狙いを定めるのは難しいため、位置を使って判定する.
        """

        # ボールからターゲットへの座標系を作成
        trans = tools.Trans(ball_pos, tools.get_angle(ball_stop_pos, ball_pos))
        tr_robot_pos = trans.transform(robot_pos)

        # ボールより前にロボットが居る場合
        if tr_robot_pos.x > 0.0:
            return False

        # ターゲットを向いていない
        if abs(tr_robot_pos.theta) > np.deg2rad(self.MINIMAL_THETA_THRESHOLD):
            return False

        if abs(tr_robot_pos.y) > self.WIDTH_THRESHOLD:
            return False

        return True
