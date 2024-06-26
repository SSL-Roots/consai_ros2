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

import math
import time

from rcst.communication import Communication
from rcst import calc
from rcst.ball import Ball
from rcst.robot import RobotDict


def test_robot_speed(rcst_comm: Communication):
    rcst_comm.send_empty_world()
    ball_x = 1.0
    rcst_comm.send_ball(ball_x, 0)
    for i in range(11):
        rcst_comm.send_blue_robot(i, -1.0, 3.0 - i * 0.5, math.radians(0))

    rcst_comm.change_referee_command('STOP', 3.0)

    rcst_comm.observer.reset()
    success = True
    rcst_comm.send_ball(ball_x, 0, 5.0, 0.0)  # Move the ball
    for _ in range(5):
        if rcst_comm.observer.robot_speed().some_blue_robots_over(1.5):
            success = False
            break
        time.sleep(1)
    assert success is True


def test_avoid_ball(rcst_comm: Communication):
    rcst_comm.send_empty_world()
    for i in range(11):
        rcst_comm.send_blue_robot(i, -1.0, 3.0 - i * 0.5, math.radians(0))

    def blue_robot_did_not_avoid_ball(
            ball: Ball, blue_robots: RobotDict, yellow_robots: RobotDict) -> bool:
        for robot in blue_robots.values():
            if calc.distance_robot_and_ball(robot, ball) < 0.4:
                return True
        return False

    rcst_comm.observer.customized().register_sticky_true_callback(
        "blue_robot_did_not_avoid_ball", blue_robot_did_not_avoid_ball)

    rcst_comm.change_referee_command('STOP', 1.0)

    def check(x: float, y: float, vx: float = 0.0, vy: float = 0.0):
        rcst_comm.send_ball(x, y, vx, vy)
        time.sleep(2)
        rcst_comm.observer.reset()

        success = True
        for _ in range(5):
            if rcst_comm.observer.customized().get_result("blue_robot_did_not_avoid_ball"):
                success = False
                break
            time.sleep(1)
        assert success is True

    check(0, 0)
    check(-3.5, 0)
    check(-3.5, 0, vx=2.0)
