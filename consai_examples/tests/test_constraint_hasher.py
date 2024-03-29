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


from consai_examples import constraint_hasher as hasher
from consai_msgs.msg import ConstraintLine
from consai_msgs.msg import ConstraintObject
from consai_msgs.msg import ConstraintPose
from consai_msgs.msg import ConstraintTheta
from consai_msgs.msg import ConstraintXY


def test_constraint_object():
    obj1 = ConstraintObject()
    obj2 = ConstraintObject()
    obj2.type = ConstraintObject.BALL
    obj2.robot_id = 1
    obj2.name = 'ball'
    assert hasher.hash_constraint_object(obj1) != hasher.hash_constraint_object(obj2)


def test_constraint_xy():
    xy1 = ConstraintXY()
    xy2 = ConstraintXY()
    xy2.value_x.append(1.0)
    xy2.value_y.append(2.0)
    xy2.object.append(ConstraintObject())
    assert hasher.hash_constraint_xy(xy1) != hasher.hash_constraint_xy(xy2)


def test_constraint_theta():
    theta1 = ConstraintTheta()
    theta2 = ConstraintTheta()
    theta2.value_theta.append(3.0)
    theta2.object.append(ConstraintObject())
    theta2.param = 1
    assert hasher.hash_constraint_theta(theta1) != hasher.hash_constraint_theta(theta2)


def test_constraint_pose():
    pose1 = ConstraintPose()
    pose2 = ConstraintPose()
    pose2.xy.value_x.append(1.0)
    pose2.theta.value_theta.append(3.0)
    pose2.offset.x = 1.0
    pose2.offset.y = 2.0
    pose2.offset.theta = 3.0
    assert hasher.hash_constraint_pose(pose1) != hasher.hash_constraint_pose(pose2)


def test_constraint_line():
    line1 = ConstraintLine()
    line2 = ConstraintLine()
    line2.p1.object.append(ConstraintObject())
    line2.p2.object.append(ConstraintObject())
    line2.p3.append(ConstraintXY())
    line2.p4.append(ConstraintXY())
    line2.offset_intersection_to_p2.append(1.0)
    assert hasher.hash_constraint_line(line1) != hasher.hash_constraint_line(line2)
