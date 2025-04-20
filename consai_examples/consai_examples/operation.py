
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

"""制約条件に関連するクラスや関数を定義するモジュール."""

from copy import deepcopy
from typing import NamedTuple

from consai_msgs.msg import (
    ConstraintLine, ConstraintObject, ConstraintPose,
    ConstraintTheta, ConstraintXY, RobotControlMsg, State2D
)

from consai_tools.hasher import robot_control_hasher


class TargetXY(NamedTuple):
    """XY座標に関する目標制約を表現するクラス."""

    constraint: ConstraintXY

    @classmethod
    def value(cls, x: float, y: float) -> 'TargetXY':
        """指定された座標値を目標とするTargetXYを生成する関数."""
        constraint = ConstraintXY()
        constraint.value_x.append(x)
        constraint.value_y.append(y)
        return cls(constraint)

    @classmethod
    def ball(cls) -> 'TargetXY':
        """ボールの位置を目標とするTargetXYを生成する関数."""
        obj_ball = ConstraintObject()
        obj_ball.type = ConstraintObject.BALL
        constraint = ConstraintXY()
        constraint.object.append(obj_ball)
        return cls(constraint)

    @classmethod
    def our_goal(cls) -> 'TargetXY':
        """自チームのゴール位置を目標とするTargetXYを生成する関数."""
        obj_goal = ConstraintObject()
        obj_goal.type = ConstraintObject.OUR_GOAL
        constraint = ConstraintXY()
        constraint.object.append(obj_goal)
        return cls(constraint)

    @classmethod
    def their_goal(cls) -> 'TargetXY':
        """相手チームのゴール位置を目標とするTargetXYを生成する関数."""
        obj_goal = ConstraintObject()
        obj_goal.type = ConstraintObject.THEIR_GOAL
        constraint = ConstraintXY()
        constraint.object.append(obj_goal)
        return cls(constraint)

    @classmethod
    def named_target(cls, name: str) -> 'TargetXY':
        """名前で指定された目標位置を持つTargetXYを生成する関数."""
        obj = ConstraintObject()
        obj.type = ConstraintObject.NAMED_TARGET
        obj.name = name

        constraint = ConstraintXY()
        constraint.object.append(obj)
        return cls(constraint)

    @classmethod
    def our_robot(cls, robot_id: int) -> 'TargetXY':
        """自チームの指定されたロボットの位置を目標とするTargetXYを生成する関数."""
        obj = ConstraintObject()
        obj.type = ConstraintObject.OUR_ROBOT
        obj.robot_id = robot_id

        constraint = ConstraintXY()
        constraint.object.append(obj)
        return cls(constraint)

    @classmethod
    def their_robot(cls, robot_id: int) -> 'TargetXY':
        """相手チームの指定されたロボットの位置を目標とするTargetXYを生成する関数."""
        obj = ConstraintObject()
        obj.type = ConstraintObject.THEIR_ROBOT
        obj.robot_id = robot_id

        constraint = ConstraintXY()
        constraint.object.append(obj)
        return cls(constraint)

    @classmethod
    def their_top_corner(cls) -> 'TargetXY':
        """相手ゴールの上隅（正規化座標）を目標とするTargetXYを生成する関数."""
        constraint = ConstraintXY()
        constraint.normalized = True
        constraint.value_x.insert(0, 1.0)
        constraint.value_y.insert(0, 1.0)
        return cls(constraint)

    @classmethod
    def their_bottom_corner(cls) -> 'TargetXY':
        """相手ゴールの下隅（正規化座標）を目標とするTargetXYを生成する関数."""
        constraint = ConstraintXY()
        constraint.normalized = True
        constraint.value_x.insert(0, 1.0)
        constraint.value_y.insert(0, -1.0)
        return cls(constraint)

    @classmethod
    def our_side_center(cls) -> 'TargetXY':
        """自チーム側のフィールド中央（正規化座標）を目標とするTargetXYを生成する関数."""
        constraint = ConstraintXY()
        constraint.normalized = True
        constraint.value_x.insert(0, -0.5)
        constraint.value_y.insert(0, 0.0)
        return cls(constraint)


class TargetTheta(NamedTuple):
    """向きに関する目標制約を表現するクラス."""

    constraint: ConstraintTheta

    @classmethod
    def value(cls, theta: float) -> 'TargetTheta':
        """指定した角度に向くTargetThetaを生成する関数."""
        constraint = ConstraintTheta()
        constraint.value_theta.append(theta)
        return cls(constraint)

    @classmethod
    def look_ball(cls) -> 'TargetTheta':
        """指定された角度を向くTargetThetaを生成する関数."""
        obj_ball = ConstraintObject()
        obj_ball.type = ConstraintObject.BALL
        constraint = ConstraintTheta()
        constraint.object.append(obj_ball)
        constraint.param = ConstraintTheta.PARAM_LOOK_TO
        return cls(constraint)

    @classmethod
    def look_their_goal(cls) -> 'TargetTheta':
        """相手ゴールの方向を向くTargetThetaを生成する."""
        obj_goal = ConstraintObject()
        obj_goal.type = ConstraintObject.THEIR_GOAL
        constraint = ConstraintTheta()
        constraint.object.append(obj_goal)
        constraint.param = ConstraintTheta.PARAM_LOOK_TO
        return cls(constraint)


class Operation():
    """ロボットの操作目標を定義する操作クラス."""

    def __init__(self, goal: RobotControlMsg = None) -> None:
        """初期化処理. 目標が指定されない場合はデフォルトの操作を生成する関数."""
        if goal:
            self._goal = goal
        else:
            self._goal = RobotControlMsg()
            self._goal.keep_control = True

    def get_goal(self) -> RobotControlMsg:
        """現在の操作目標を取得する関数."""
        return self._goal

    def get_hash(self) -> int:
        """操作目標のハッシュ値を取得する関数."""
        return robot_control_hasher.hash_robot_control(self._goal)

    def halt(self) -> 'Operation':
        """ロボットを停止する操作を返す関数."""
        goal = deepcopy(self._goal)
        goal.stop = True
        return Operation(goal)

    def enable_avoid_placement_area(self, placement_pos: State2D) -> 'Operation':
        """配置エリアの回避を有効にする関数."""
        goal = deepcopy(self._goal)
        goal.avoid_placement_area = True
        goal.placement_pos = placement_pos
        return Operation(goal)

    def enable_avoid_ball(self) -> 'Operation':
        """ボールの回避を有効にする関数."""
        goal = deepcopy(self._goal)
        goal.avoid_ball = True
        return Operation(goal)

    def enable_avoid_pushing_robots(self) -> 'Operation':
        """押し合いを避ける回避動作を有効にする関数."""
        goal = deepcopy(self._goal)
        goal.avoid_pushing_robots = True
        return Operation(goal)

    def disable_avoid_defense_area(self) -> 'Operation':
        """ディフェンスエリアの回避を無効にする関数."""
        goal = deepcopy(self._goal)
        goal.avoid_defense_area = False
        return Operation(goal)

    def disable_avoid_our_robots(self) -> 'Operation':
        """味方ロボットの回避を無効にする関数."""
        goal = deepcopy(self._goal)
        goal.avoid_our_robots = False
        return Operation(goal)

    def disable_avoid_their_robots(self) -> 'Operation':
        """相手ロボットの回避を無効にする関数."""
        goal = deepcopy(self._goal)
        goal.avoid_their_robots = False
        return Operation(goal)

    def restrict_velocity_xy(self, velocity: float) -> 'Operation':
        """最大XY速度を制限する関数."""
        goal = deepcopy(self._goal)
        goal.max_velocity_xy.insert(0, velocity)
        return Operation(goal)

    def move_on_line(self, p1: TargetXY, p2: TargetXY, distance_from_p1: float,
                     target_theta: TargetTheta) -> 'Operation':
        """指定された直線上の距離と角度に基づいて移動する操作を生成する関数."""
        line = ConstraintLine()
        line.p1 = p1.constraint
        line.p2 = p2.constraint
        line.distance = distance_from_p1
        line.theta = target_theta.constraint
        goal = deepcopy(self._goal)
        goal.line.insert(0, line)
        return Operation(goal)

    def move_to_intersection(self, p1: TargetXY, p2: TargetXY, p3: TargetXY, p4: TargetXY,
                             target_theta: TargetTheta, offset: float = 0.0) -> 'Operation':
        """2本の直線の交点へ移動する操作を生成する関数."""
        line = ConstraintLine()
        line.p1 = p1.constraint
        line.p2 = p2.constraint
        line.p3.insert(0, p3.constraint)
        line.p4.insert(0, p4.constraint)
        line.offset_intersection_to_p2.insert(0, offset)
        line.theta = target_theta.constraint
        goal = deepcopy(self._goal)
        goal.line.insert(0, line)
        return Operation(goal)

    def offset_intersection_to_p2(self, offset: float) -> 'Operation':
        """交点からp2へのオフセットを設定する関数."""
        goal = deepcopy(self._goal)
        if goal.line:
            if goal.line[0].offset_intersection_to_p2:
                goal.line[0].offset_intersection_to_p2[0] = offset
        return Operation(goal)

    def move_to_pose(self, target_xy: TargetXY, target_theta: TargetTheta) -> 'Operation':
        """指定された姿勢へ移動する操作を生成する関数."""
        pose = ConstraintPose()
        pose.xy = target_xy.constraint
        pose.theta = target_theta.constraint
        goal = deepcopy(self._goal)
        goal.pose.insert(0, pose)
        return Operation(goal)

    def overwrite_pose_x(self, value: float) -> 'Operation':
        """現在の目標姿勢のX座標を上書きする関数."""
        goal = deepcopy(self._goal)
        if goal.pose:
            goal.pose[0].xy.value_x.insert(0, value)
        return Operation(goal)

    def overwrite_pose_y(self, value: float) -> 'Operation':
        """現在の目標姿勢のY座標を上書きする関数."""
        goal = deepcopy(self._goal)
        if goal.pose:
            goal.pose[0].xy.value_y.insert(0, value)
        return Operation(goal)

    def overwrite_pose_theta(self, value: float) -> 'Operation':
        """現在の目標姿勢の角度を上書きする関数."""
        goal = deepcopy(self._goal)
        if goal.pose:
            goal.pose[0].theta.value_theta.insert(0, value)
        return Operation(goal)

    def offset_pose_x(self, offset: float) -> 'Operation':
        """現在の目標姿勢のX方向にオフセットを追加する関数."""
        goal = deepcopy(self._goal)
        if goal.pose:
            goal.pose[0].offset.x = offset
        return Operation(goal)

    def offset_pose_y(self, offset: float) -> 'Operation':
        """現在の目標姿勢のY方向にオフセットを追加する関数."""
        goal = deepcopy(self._goal)
        if goal.pose:
            goal.pose[0].offset.y = offset
        return Operation(goal)

    def offset_pose_theta(self, offset: float) -> 'Operation':
        """現在の目標姿勢の角度にオフセットを追加する関数."""
        goal = deepcopy(self._goal)
        if goal.pose:
            goal.pose[0].offset.theta = offset
        return Operation(goal)

    def with_ball_receiving(self) -> 'Operation':
        """ボールを受け取る動作を有効にする関数."""
        goal = deepcopy(self._goal)
        goal.receive_ball = True
        return Operation(goal)

    def with_shooting_to(self, target_xy: TargetXY) -> 'Operation':
        """指定位置に向けてシュートする操作を設定する関数."""
        goal = deepcopy(self._goal)
        goal.kick_enable = True
        goal.kick_target = target_xy.constraint
        return Operation(goal)

    def with_shooting_for_setplay_to(self, target_xy: TargetXY) -> 'Operation':
        """セットプレーとして指定位置にシュートする操作を設定する関数."""
        goal = deepcopy(self._goal)
        goal.kick_enable = True
        goal.kick_setplay = True
        goal.kick_target = target_xy.constraint
        return Operation(goal)

    def with_passing_to(self, target_xy: TargetXY) -> 'Operation':
        """指定位置にパスする操作を設定する関数."""
        goal = deepcopy(self._goal)
        goal.kick_enable = True
        goal.kick_pass = True
        goal.kick_target = target_xy.constraint
        return Operation(goal)

    def with_passing_for_setplay_to(self, target_xy: TargetXY) -> 'Operation':
        """セットプレーとして指定位置にパスする操作を設定する関数."""
        goal = deepcopy(self._goal)
        goal.kick_enable = True
        goal.kick_pass = True
        goal.kick_setplay = True
        goal.kick_target = target_xy.constraint
        return Operation(goal)

    def with_dribbling_to(self, target_xy: TargetXY) -> 'Operation':
        """指定位置へドリブルする操作を設定する関数."""
        goal = deepcopy(self._goal)
        goal.dribble_enable = True
        goal.dribble_target = target_xy.constraint
        return Operation(goal)

    def with_back_dribbling_to(self, target_xy: TargetXY) -> 'Operation':
        """指定位置へバックドリブルする操作を設定する関数."""
        goal = deepcopy(self._goal)
        goal.back_dribble_enable = True
        goal.dribble_target = target_xy.constraint
        return Operation(goal)

    def with_reflecting_to(self, target_xy: TargetXY) -> 'Operation':
        """指定位置に向けて反射シュートする操作を設定する関数."""
        goal = deepcopy(self._goal)
        goal.reflect_shoot = True
        goal.kick_target = target_xy.constraint
        return Operation(goal)

    def with_ball_boy_dribbling_to(self, target_xy: TargetXY) -> 'Operation':
        """ボールボーイ動作として指定位置へドリブルする操作を設定する関数."""
        goal = deepcopy(self._goal)
        goal.ball_boy_dribble_enable = True
        goal.dribble_target = target_xy.constraint
        return Operation(goal)


class OneShotOperation(Operation):
    """一度だけの操作を定義するクラス. 指定姿勢に到達したら制御を終了する関数."""

    def __init__(self, goal: RobotControlMsg = None) -> None:
        """初期化処理を行い, 目標姿勢に到達したら制御を終了する設定をする."""
        super().__init__(goal)

        # 目標姿勢にたどり着いたら制御を終える
        self._goal.keep_control = False
