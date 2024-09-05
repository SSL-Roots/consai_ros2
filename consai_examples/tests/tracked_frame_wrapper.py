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

from robocup_ssl_msgs.msg import RobotId
from robocup_ssl_msgs.msg import TrackedBall
from robocup_ssl_msgs.msg import TrackedFrame
from robocup_ssl_msgs.msg import TrackedRobot


class TrackedFrameWrapper():
    def __init__(self):
        self._frame = TrackedFrame()

    def append_valid_robots(self, blue_robots: list[int], yellow_robots: list[int]):
        self._append_robots(blue_robots, yellow_robots, 1.0)

    def append_invalid_robots(self, blue_robots: list[int], yellow_robots: list[int]):
        self._append_robots(blue_robots, yellow_robots, 0.0)

    def _append_robots(self, blue_robots: list[int], yellow_robots: list[int], visibility: float):
        for blue_id in blue_robots:
            robot = TrackedRobot()
            robot.robot_id.id = blue_id
            robot.robot_id.team_color = RobotId.TEAM_COLOR_BLUE
            robot.visibility.append(visibility)
            self._frame.robots.append(robot)

        for yellow_id in yellow_robots:
            robot = TrackedRobot()
            robot.robot_id.id = yellow_id
            robot.robot_id.team_color = RobotId.TEAM_COLOR_YELLOW
            robot.visibility.append(visibility)
            self._frame.robots.append(robot)

    def append_valid_ball(self, pos_x: float, pos_y: float):
        ball = TrackedBall()
        ball.visibility.append(1.0)
        ball.pos.x = pos_x
        ball.pos.y = pos_y
        self._frame.balls.append(ball)

    def get_frame(self) -> TrackedFrame:
        return self._frame
