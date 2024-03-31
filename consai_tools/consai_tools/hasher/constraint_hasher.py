# Copyright 2024 Roots
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


from consai_msgs.msg import ConstraintLine
from consai_msgs.msg import ConstraintObject
from consai_msgs.msg import ConstraintPose
from consai_msgs.msg import ConstraintTheta
from consai_msgs.msg import ConstraintXY


def hash_constraint_object(obj: ConstraintObject) -> int:
    target = (
        obj.type,
        obj.robot_id,
        obj.name
    )
    return hash(target)


def hash_constraint_xy(xy: ConstraintXY) -> int:
    object_hash = tuple(hash_constraint_object(obj) for obj in xy.object)

    target = (
        xy.normalized,
        tuple(xy.value_x),
        tuple(xy.value_y),
        object_hash
    )
    return hash(target)


def hash_constraint_theta(theta: ConstraintTheta) -> int:
    object_hash = tuple(hash_constraint_object(obj) for obj in theta.object)

    target = (
        tuple(theta.value_theta),
        object_hash,
        theta.param
    )
    return hash(target)


def hash_constraint_pose(pose: ConstraintPose) -> int:
    xy_hash = hash_constraint_xy(pose.xy)
    theta_hash = hash_constraint_theta(pose.theta)

    target = (
        xy_hash,
        theta_hash,
        pose.offset.x,
        pose.offset.y,
        pose.offset.theta
    )
    return hash(target)


def hash_constraint_line(line: ConstraintLine) -> int:
    p1_hash = hash_constraint_xy(line.p1)
    p2_hash = hash_constraint_xy(line.p2)
    theta_hash = hash_constraint_theta(line.theta)
    p3_hash = tuple(hash_constraint_xy(p) for p in line.p3)
    p4_hash = tuple(hash_constraint_xy(p) for p in line.p4)

    target = (
        p1_hash,
        p2_hash,
        line.distance,
        theta_hash,
        p3_hash,
        p4_hash,
        tuple(line.offset_intersection_to_p2)
    )
    return hash(target)
