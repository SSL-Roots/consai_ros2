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


from consai_msgs.action import RobotControl
from consai_examples import constraint_hasher as hasher


def hash_goal(goal: RobotControl.Goal) -> int:
    pose_hash = tuple(hasher.hash_constraint_pose(pose) for pose in goal.pose)
    line_hash = tuple(hasher.hash_constraint_line(line) for line in goal.line)

    target = (
        goal.stop,
        goal.keep_control,
        tuple(goal.max_velocity_xy),
        pose_hash,
        line_hash,
        goal.kick_enable,
        goal.kick_pass,
        goal.kick_setplay,
        hasher.hash_constraint_xy(goal.kick_target),
        goal.dribble_enable,
        goal.ball_boy_dribble_enable,
        hasher.hash_constraint_xy(goal.dribble_target),
        goal.receive_ball,
        goal.reflect_shoot,
        goal.avoid_obstacles,
        goal.avoid_placement_area,
        goal.placement_pos.x,
        goal.placement_pos.y,
        goal.avoid_ball,
    )
    return hash(target)
