import numpy as np

from consai_game.utils.geometry import Point
from consai_game.world_model.ball_model import BallModel
from consai_game.world_model.robots_model import Robot

from consai_msgs.msg import State2D

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


class RobotDecision:
    """ロボットやボールの位置関係を判定するクラス."""

    def obstacle_exists(target: State2D, ball: BallModel, robots: dict[int, Robot], tolerance) -> bool:
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

    def is_robot_backside(
        robot_pos: State2D, ball_pos: State2D, target_pos: State2D, angle_ball_to_robot_threshold: int
    ) -> bool:
        """ボールからターゲットを見て、ロボットが後側に居るかを判定するメソッド."""

        # ボールからターゲットへの座標系を作成
        trans = tools.Trans(ball_pos, tools.get_angle(ball_pos, target_pos))
        tr_robot_pos = trans.transform(robot_pos)

        # ボールから見たロボットの位置の角度
        # ボールの後方にいれば角度は90度以上
        tr_ball_to_robot_angle = tools.get_angle(State2D(x=0.0, y=0.0), tr_robot_pos)

        if abs(tr_ball_to_robot_angle) > np.deg2rad(angle_ball_to_robot_threshold):
            return True
        return False

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

    def is_ball_front(robot_pos: State2D, ball_pos: State2D, target_pos: State2D) -> bool:
        """ボールがロボットの前にあるかどうかを判定するメソッド."""

        front_dist_threshold = 0.15  # 正面方向にどれだけ離れることを許容するか
        side_dist_threshold = 0.05  # 横方向にどれだけ離れることを許容するか

        # ロボットを中心に、ターゲットを+x軸とした座標系を作る
        trans = tools.Trans(robot_pos, tools.get_angle(robot_pos, target_pos))
        tr_ball_pos = trans.transform(ball_pos)

        # ボールがロボットの後ろにある
        if tr_ball_pos.x < 0:
            return False

        # ボールが正面から離れすぎている
        if tr_ball_pos.x > front_dist_threshold:
            return False

        # ボールが横方向に離れすぎている
        if abs(tr_ball_pos.y) > side_dist_threshold:
            return False
        return True

    def update_our_robots_arrived(self, our_visible_robots: dict[int, Robot], commands: list[MotionCommand]) -> bool:
        """各ロボットが目標位置に到達したか判定する関数."""

        # ロボットが目標位置に到着したと判定する距離[m]
        DIST_ROBOT_TO_DESIRED_THRESHOLD = 0.1

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
                    arrived=dist_robot_to_desired < DIST_ROBOT_TO_DESIRED_THRESHOLD,
                )
            )
