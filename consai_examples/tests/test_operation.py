# Copyright 2023 Roots
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


from consai_examples.operation import Operation
from consai_examples.operation import OneShotOperation
from consai_examples.operation import PointXY
from consai_msgs.msg import ConstraintObject
from consai_msgs.msg import ConstraintTheta
import pytest


def test_move_to_ball_position():
    goal = Operation().move_to_position(PointXY.ball()).get_goal()
    assert len(goal.pose) == 1
    assert len(goal.pose[0].xy.object) == 1
    assert goal.pose[0].xy.object[0].type == ConstraintObject.BALL


def test_set_pose_x_y_theta():
    operation = Operation().move_to_position(PointXY.ball())
    operation = operation.set_pose_x(1.0)
    operation = operation.set_pose_y(2.0)
    operation = operation.set_pose_theta(3.0)
    goal = operation.get_goal()
    assert len(goal.pose[0].xy.value_x) == 1
    assert len(goal.pose[0].xy.value_y) == 1
    assert len(goal.pose[0].theta.value_theta) == 1
    assert goal.pose[0].xy.value_x[0] == 1.0
    assert goal.pose[0].xy.value_y[0] == 2.0
    assert goal.pose[0].theta.value_theta[0] == 3.0


def test_immutability():
    operation = Operation().move_to_position(PointXY.ball())
    operation.set_pose_x(1.0)
    operation.set_pose_y(2.0)
    operation.set_pose_theta(3.0)
    goal = operation.get_goal()
    assert len(goal.pose[0].xy.value_x) == 0
    assert len(goal.pose[0].xy.value_y) == 0
    assert len(goal.pose[0].theta.value_theta) == 0


def test_set_pose_theta_to_look_ball():
    operation = Operation().move_to_position(PointXY.ball())
    operation = operation.set_pose_theta_to_look_ball()
    goal = operation.get_goal()

    assert len(goal.pose[0].theta.object) == 1
    assert goal.pose[0].xy.object[0].type == ConstraintObject.BALL
    assert goal.pose[0].theta.object[0].type == ConstraintObject.BALL
    assert goal.pose[0].theta.param == ConstraintTheta.PARAM_LOOK_TO


def test_offset_pose_x_y_theta():
    operation = Operation().move_to_position(PointXY.ball())
    operation = operation.offset_pose_x(1.0)
    operation = operation.offset_pose_y(2.0)
    operation = operation.offset_pose_theta(3.0)
    goal = operation.get_goal()
    assert goal.pose[0].offset.x == 1.0
    assert goal.pose[0].offset.y == 2.0
    assert goal.pose[0].offset.theta == 3.0


def test_with_ball_receiving():
    operation = Operation().move_to_position(PointXY.ball())
    operation = operation.with_ball_receiving()
    goal = operation.get_goal()
    assert goal.receive_ball is True


def test_with_shooting_to():
    operation = Operation().move_to_position(PointXY.ball())
    operation = operation.with_shooting_to(PointXY.their_goal())
    goal = operation.get_goal()
    assert goal.kick_enable is True
    assert goal.kick_pass is False
    assert goal.kick_target.normalized is True
    assert goal.kick_target.value_x[0] == 1.0
    assert goal.kick_target.value_y[0] == 0.0


def test_with_shooting_carefully_to():
    operation = Operation().move_to_position(PointXY.ball())
    operation = operation.with_shooting_carefully_to(PointXY.their_goal())
    goal = operation.get_goal()
    assert goal.kick_enable is True
    assert goal.kick_pass is False
    assert goal.kick_setplay is True
    assert goal.kick_target.normalized is True
    assert goal.kick_target.value_x[0] == 1.0
    assert goal.kick_target.value_y[0] == 0.0


def test_with_passing_to():
    operation = Operation().move_to_position(PointXY.ball())
    operation = operation.with_passing_to(PointXY.named_target("target"))
    goal = operation.get_goal()
    assert goal.kick_enable is True
    assert goal.kick_pass is True
    assert goal.kick_target.object[0].type == ConstraintObject.NAMED_TARGET
    assert goal.kick_target.object[0].name == "target"


def test_with_reflecting_kick():
    operation = Operation().move_to_position(PointXY.ball())
    operation = operation.with_reflecting_kick()
    goal = operation.get_goal()
    assert goal.reflect_shoot is True


def test_keep_control_operation():
    goal = Operation().move_to_position(PointXY.ball()).get_goal()
    assert len(goal.pose) == 1
    assert len(goal.pose[0].xy.object) == 1
    assert goal.pose[0].xy.object[0].type == ConstraintObject.BALL

    assert goal.keep_control is True


def test_oneshot_operation():
    goal = OneShotOperation().move_to_position(PointXY.ball()).get_goal()
    assert len(goal.pose) == 1
    assert len(goal.pose[0].xy.object) == 1
    assert goal.pose[0].xy.object[0].type == ConstraintObject.BALL

    assert goal.keep_control is False
