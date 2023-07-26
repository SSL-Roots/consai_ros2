
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

from consai_msgs.action import RobotControl
from consai_msgs.msg import ConstraintObject
from consai_msgs.msg import ConstraintPose
from consai_msgs.msg import ConstraintTheta
from consai_msgs.msg import ConstraintXY
from copy import deepcopy
from typing import NamedTuple


class TargetXY(NamedTuple):
    constraint: ConstraintXY

    @classmethod
    def value(cls, x: float, y: float) -> 'TargetXY':
        constraint = ConstraintXY()
        constraint.value_x.append(x)
        constraint.value_y.append(y)
        return cls(constraint)

    @classmethod
    def ball(cls) -> 'TargetXY':
        obj_ball = ConstraintObject()
        obj_ball.type = ConstraintObject.BALL
        constraint = ConstraintXY()
        constraint.object.append(obj_ball)
        return cls(constraint)

    @classmethod
    def their_goal(cls) -> 'TargetXY':
        constraint = ConstraintXY()
        constraint.normalized = True
        constraint.value_x.append(1.0)
        constraint.value_y.append(0.0)
        return cls(constraint)

    @classmethod
    def named_target(cls, name: str) -> 'TargetXY':
        obj = ConstraintObject()
        obj.type = ConstraintObject.NAMED_TARGET
        obj.name = name

        constraint = ConstraintXY()
        constraint.object.append(obj)
        return cls(constraint)


class TargetTheta(NamedTuple):
    constraint: ConstraintTheta

    @classmethod
    def look_ball(cls) -> 'TargetTheta':
        obj_ball = ConstraintObject()
        obj_ball.type = ConstraintObject.BALL
        constraint = ConstraintTheta()
        constraint.object.append(obj_ball)
        constraint.param = ConstraintTheta.PARAM_LOOK_TO
        return cls(constraint)


class Operation():
    def __init__(self, goal: RobotControl.Goal = None) -> None:
        if goal:
            self._goal = goal
        else:
            self._goal = RobotControl.Goal()
            self._goal.keep_control = True

    def get_goal(self) -> RobotControl.Goal:
        return self._goal

    def move_to_pose(self, target_xy: TargetXY, target_theta: TargetTheta):
        pose = ConstraintPose()
        pose.xy = target_xy.constraint
        pose.theta = target_theta.constraint
        goal = deepcopy(self._goal)
        goal.pose.insert(0, pose)
        return Operation(goal)

    def overwrite_pose_x(self, value: float):
        goal = deepcopy(self._goal)
        if goal.pose:
            goal.pose[0].xy.value_x.insert(0, value)
        return Operation(goal)

    def overwrite_pose_y(self, value: float):
        goal = deepcopy(self._goal)
        if goal.pose:
            goal.pose[0].xy.value_y.insert(0, value)
        return Operation(goal)

    def overwrite_pose_theta(self, value: float):
        goal = deepcopy(self._goal)
        if goal.pose:
            goal.pose[0].theta.value_theta.insert(0, value)
        return Operation(goal)

    def offset_pose_x(self, offset: float):
        goal = deepcopy(self._goal)
        if goal.pose:
            goal.pose[0].offset.x = offset
        return Operation(goal)

    def offset_pose_y(self, offset: float):
        goal = deepcopy(self._goal)
        if goal.pose:
            goal.pose[0].offset.y = offset
        return Operation(goal)

    def offset_pose_theta(self, offset: float):
        goal = deepcopy(self._goal)
        if goal.pose:
            goal.pose[0].offset.theta = offset
        return Operation(goal)

    def with_ball_receiving(self):
        goal = deepcopy(self._goal)
        goal.receive_ball = True
        return Operation(goal)

    def with_shooting_to(self, target_xy: TargetXY):
        goal = deepcopy(self._goal)
        goal.kick_enable = True
        goal.kick_target = target_xy.constraint
        return Operation(goal)

    def with_shooting_carefully_to(self, target_xy: TargetXY):
        goal = deepcopy(self._goal)
        goal.kick_enable = True
        goal.kick_setplay = True
        goal.kick_target = target_xy.constraint
        return Operation(goal)

    def with_passing_to(self, target_xy: TargetXY):
        goal = deepcopy(self._goal)
        goal.kick_enable = True
        goal.kick_pass = True
        goal.kick_target = target_xy.constraint
        return Operation(goal)

    def with_reflecting_kick(self):
        goal = deepcopy(self._goal)
        goal.reflect_shoot = True
        return Operation(goal)

    def _object_ball(self):
        obj_ball = ConstraintObject()
        obj_ball.type = ConstraintObject.BALL
        return obj_ball


class OneShotOperation(Operation):
    def __init__(self, goal: RobotControl.Goal = None) -> None:
        super().__init__(goal)

        # 目標姿勢にたどり着いたら制御を終える
        self._goal.keep_control = False
