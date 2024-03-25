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


from consai_examples import robot_control_hasher as hasher
from consai_msgs.action import RobotControl
from consai_msgs.msg import ConstraintLine
from consai_msgs.msg import ConstraintPose


def test_hash_robot_control():
    goal1 = RobotControl.Goal()
    goal2 = RobotControl.Goal()
    # リスト形式のものだけ個別に値をセットする
    goal2.pose.append(ConstraintPose())
    goal2.line.append(ConstraintLine())

    assert hasher.hash_robot_control(goal1) != hasher.hash_robot_control(goal2)
