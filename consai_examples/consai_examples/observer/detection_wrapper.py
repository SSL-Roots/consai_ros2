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


from consai_examples.observer.pos_vel import PosVel
from robocup_ssl_msgs.msg import RobotId
from robocup_ssl_msgs.msg import TrackedFrame
from robocup_ssl_msgs.msg import TrackedRobot
from robocup_ssl_msgs.msg import TrackedBall


class DetectionWrapper:
    def __init__(self, our_team_is_yellow: bool, visibility_threshold=0.01):
        self._our_team_is_yellow = our_team_is_yellow
        self._visibility_threshold = visibility_threshold

        self._visible_ball = PosVel()
        self._visible_blue_robots: dict[int, PosVel] = {}
        self._visible_yellow_robots: dict[int, PosVel] = {}
        self._our_robots: dict[int, PosVel] = {}
        self._their_robots: dict[int, PosVel] = {}

    def update(self, detection_tracked: TrackedFrame):

        for ball in detection_tracked.balls:
            if len(ball.visibility) <= 0:
                continue
            if ball.visibility[0] < self._visibility_threshold:
                continue
            self._visible_ball = self._extract_ball(ball)

        for robot in detection_tracked.robots:
            if len(robot.visibility) <= 0:
                continue
            if robot.visibility[0] < self._visibility_threshold:
                continue
            pos_vel = self._extract_robot(robot)

            if robot.robot_id.team_color == RobotId.TEAM_COLOR_BLUE:
                self._visible_blue_robots[robot.robot_id.id] = pos_vel
            else:
                self._visible_yellow_robots[robot.robot_id.id] = pos_vel

        if self._our_team_is_yellow:
            self._our_robots = self._visible_yellow_robots
            self._their_robots = self._visible_blue_robots
        else:
            self._our_robots = self._visible_blue_robots
            self._their_robots = self._visible_yellow_robots

    def ball(self) -> PosVel:
        return self._visible_ball

    def our_robots(self) -> dict[int, PosVel]:
        return self._our_robots

    def their_robots(self) -> dict[int, PosVel]:
        return self._their_robots

    def _extract_ball(self, ball: TrackedBall) -> PosVel:
        pos_vel = PosVel()
        pos_vel.set_pos(ball.pos.x, ball.pos.y)
        if len(ball.vel) > 0:
            pos_vel.set_vel(ball.vel[0].x, ball.vel[0].y)
        return pos_vel

    def _extract_robot(self, robot: TrackedRobot) -> PosVel:
        pos_vel = PosVel()
        pos_vel.set_pos(robot.pos.x, robot.pos.y, robot.orientation)
        if len(robot.vel) > 0 and len(robot.vel_angular) > 0:
            pos_vel.set_vel(robot.vel[0].x, robot.vel[0].y, robot.vel_angular[0])
        return pos_vel
