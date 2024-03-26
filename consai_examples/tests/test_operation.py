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
from consai_examples.operation import TargetXY
from consai_examples.operation import TargetTheta
from consai_msgs.msg import ConstraintObject
from consai_msgs.msg import ConstraintTheta


def test_get_hash():
    operation = Operation().move_to_pose(TargetXY.ball(), TargetTheta.look_ball())
    hash1 = operation.get_hash()
    hash2 = operation.get_hash()
    assert hash1 == hash2

    operation = OneShotOperation().move_to_pose(TargetXY.ball(), TargetTheta.look_ball())
    hash3 = operation.get_hash()
    assert hash1 != hash3


def test_stop():
    goal = Operation().get_goal()
    assert goal.stop is False

    goal = Operation().stop().get_goal()
    assert goal.stop is True


def test_immutability():
    operation = Operation().move_to_pose(TargetXY.ball(), TargetTheta.look_ball())
    operation.overwrite_pose_x(1.0)
    operation.overwrite_pose_y(2.0)
    operation.overwrite_pose_theta(3.0)
    goal = operation.get_goal()
    assert len(goal.pose[0].xy.value_x) == 0
    assert len(goal.pose[0].xy.value_y) == 0
    assert len(goal.pose[0].theta.value_theta) == 0


def test_move_on_line():
    goal = Operation().move_on_line(TargetXY.ball(), TargetXY.their_goal(), 0.5,
                                    TargetTheta.look_ball()).get_goal()
    assert len(goal.line) >= 1
    assert goal.line[0].p1.object[0].type == ConstraintObject.BALL
    assert goal.line[0].p2.object[0].type == ConstraintObject.THEIR_GOAL
    assert goal.line[0].distance == 0.5
    assert goal.line[0].theta.object[0].type == ConstraintObject.BALL


def test_move_to_intersection():
    goal = Operation().move_to_intersection(
        TargetXY.our_goal(), TargetXY.ball(),
        TargetXY.our_robot(0), TargetXY.our_robot(1), TargetTheta.look_ball()).get_goal()
    assert len(goal.line) >= 1
    assert goal.line[0].p1.object[0].type == ConstraintObject.OUR_GOAL
    assert goal.line[0].p2.object[0].type == ConstraintObject.BALL
    assert len(goal.line[0].p3) >= 1
    assert goal.line[0].p3[0].object[0].type == ConstraintObject.OUR_ROBOT
    assert len(goal.line[0].p4) >= 1
    assert goal.line[0].p4[0].object[0].type == ConstraintObject.OUR_ROBOT
    assert len(goal.line[0].offset_intersection_to_p2) >= 1
    assert goal.line[0].offset_intersection_to_p2[0] == 0.0
    assert goal.line[0].theta.object[0].type == ConstraintObject.BALL


def test_offset_intersection_to_p2():
    operation = Operation().move_to_intersection(
        TargetXY.our_goal(), TargetXY.ball(),
        TargetXY.our_robot(0), TargetXY.our_robot(1), TargetTheta.look_ball())
    goal = operation.offset_intersection_to_p2(0.5).get_goal()
    assert len(goal.line[0].offset_intersection_to_p2) >= 1
    assert goal.line[0].offset_intersection_to_p2[0] == 0.5


def test_move_to_ball_pose():
    goal = Operation().move_to_pose(TargetXY.ball(), TargetTheta.look_ball()).get_goal()
    assert len(goal.pose) >= 1
    assert len(goal.pose[0].xy.object) >= 1
    assert goal.pose[0].xy.object[0].type == ConstraintObject.BALL
    assert goal.pose[0].theta.object[0].type == ConstraintObject.BALL
    assert goal.pose[0].theta.param == ConstraintTheta.PARAM_LOOK_TO


def test_overwrite_pose_x_y_theta():
    operation = Operation().move_to_pose(TargetXY.value(-1.0, -2.0), TargetTheta.look_ball())
    operation = operation.overwrite_pose_x(1.0)
    operation = operation.overwrite_pose_y(2.0)
    operation = operation.overwrite_pose_theta(3.0)
    goal = operation.get_goal()
    assert len(goal.pose[0].xy.value_x) >= 1
    assert len(goal.pose[0].xy.value_y) >= 1
    assert len(goal.pose[0].theta.value_theta) >= 1
    assert goal.pose[0].xy.value_x[0] == 1.0
    assert goal.pose[0].xy.value_y[0] == 2.0
    assert goal.pose[0].theta.value_theta[0] == 3.0


def test_offset_pose_x_y_theta():
    operation = Operation().move_to_pose(TargetXY.ball(), TargetTheta.look_ball())
    operation = operation.offset_pose_x(1.0)
    operation = operation.offset_pose_y(2.0)
    operation = operation.offset_pose_theta(3.0)
    goal = operation.get_goal()
    assert goal.pose[0].offset.x == 1.0
    assert goal.pose[0].offset.y == 2.0
    assert goal.pose[0].offset.theta == 3.0


def test_with_ball_receiving():
    operation = Operation().move_to_pose(TargetXY.ball(), TargetTheta.look_ball())
    operation = operation.with_ball_receiving()
    goal = operation.get_goal()
    assert goal.receive_ball is True


def test_with_shooting_to():
    operation = Operation().move_to_pose(TargetXY.ball(), TargetTheta.look_ball())
    operation = operation.with_shooting_to(TargetXY.their_goal())
    goal = operation.get_goal()
    assert goal.kick_enable is True
    assert goal.kick_pass is False
    assert goal.kick_target.object[0].type == ConstraintObject.THEIR_GOAL


def test_with_shooting_for_setplay_to():
    operation = Operation().move_to_pose(TargetXY.ball(), TargetTheta.look_ball())
    operation = operation.with_shooting_for_setplay_to(TargetXY.their_goal())
    goal = operation.get_goal()
    assert goal.kick_enable is True
    assert goal.kick_pass is False
    assert goal.kick_setplay is True
    assert goal.kick_target.object[0].type == ConstraintObject.THEIR_GOAL


def test_with_passing_to():
    operation = Operation().move_to_pose(TargetXY.ball(), TargetTheta.look_ball())
    operation = operation.with_passing_to(TargetXY.named_target("target"))
    goal = operation.get_goal()
    assert goal.kick_enable is True
    assert goal.kick_pass is True
    assert goal.kick_target.object[0].type == ConstraintObject.NAMED_TARGET
    assert goal.kick_target.object[0].name == "target"


def test_with_dribbling_to():
    operation = Operation().move_to_pose(TargetXY.ball(), TargetTheta.look_ball())
    operation = operation.with_dribbling_to(TargetXY.our_robot(3))
    goal = operation.get_goal()
    assert goal.dribble_enable is True
    assert goal.dribble_target.object[0].type == ConstraintObject.OUR_ROBOT
    assert goal.dribble_target.object[0].robot_id == 3


def test_with_reflecting_to():
    operation = Operation().move_to_pose(TargetXY.ball(), TargetTheta.look_ball())
    operation = operation.with_reflecting_to(TargetXY.their_goal())
    goal = operation.get_goal()
    assert goal.reflect_shoot is True
    assert goal.kick_target.object[0].type == ConstraintObject.THEIR_GOAL


def test_with_ball_boy_dribbling_to():
    operation = Operation().move_to_pose(TargetXY.ball(), TargetTheta.look_ball())
    operation = operation.with_ball_boy_dribbling_to(TargetXY.our_robot(3))
    goal = operation.get_goal()
    assert goal.ball_boy_dribble_enable is True
    assert goal.dribble_target.object[0].type == ConstraintObject.OUR_ROBOT
    assert goal.dribble_target.object[0].robot_id == 3


def test_keep_control_operation():
    goal = Operation().move_to_pose(TargetXY.ball(), TargetTheta.look_ball()).get_goal()
    assert len(goal.pose) >= 1
    assert len(goal.pose[0].xy.object) >= 1
    assert goal.pose[0].xy.object[0].type == ConstraintObject.BALL

    assert goal.keep_control is True


def test_oneshot_operation():
    goal = OneShotOperation().move_to_pose(TargetXY.ball(), TargetTheta.look_ball()).get_goal()
    assert len(goal.pose) >= 1
    assert len(goal.pose[0].xy.object) >= 1
    assert goal.pose[0].xy.object[0].type == ConstraintObject.BALL

    assert goal.keep_control is False