from consai_game.core.tactic.tactic_base import TacticBase
from consai_msgs.msg import MotionCommand
from consai_msgs.msg import State2D
from consai_game.world_model.world_model import WorldModel
from consai_tools.geometry import geometry_tools as tool
import math


class ManMark(TacticBase):
    """指定された敵ロボットを一定距離を保ってマークするTactic."""

    def __init__(self, target_robot_id: int, mark_distance: float = 0.5):
        """
        Args:
            target_robot_id: マーク対象の敵ロボットID
            mark_distance: 対象からどれくらい離れてマークするか（m）
        """
        super().__init__()
        self.target_robot_id = target_robot_id
        self.mark_distance = mark_distance

    def reset(self, robot_id: int):
        self.robot_id = robot_id

    def run(self, world_model: WorldModel) -> MotionCommand:
        command = MotionCommand()
        command.robot_id = self.robot_id
        command.mode = MotionCommand.MODE_NAVI

        target_robot = world_model.robots.their_robots.get(self.target_robot_id)

        if not target_robot or not target_robot.is_visible:
            # ターゲットが見えなければ止まる
            command.mode = MotionCommand.MODE_DIRECT_VELOCITY
            command.desired_velocity.x = 0.0
            command.desired_velocity.y = 0.0
            command.desired_velocity.theta = 0.0
            return command

        # 角度を計算（ゴール → 敵）
        angle_to_target = tool.get_angle(State2D(x=world_model.field_points.our_goal_top.x, y=0.0), target_robot.pos)

        # 敵との間合いを取る位置を計算（敵から mark_distance 離れたところ）
        mark_x = target_robot.pos.x - self.mark_distance * math.cos(angle_to_target)
        mark_y = target_robot.pos.y - self.mark_distance * math.sin(angle_to_target)

        # 目的地設定
        command.desired_pose = State2D()
        command.desired_pose.x = mark_x
        command.desired_pose.y = mark_y
        command.desired_pose.theta = angle_to_target  # 敵の方向を向く

        return command
