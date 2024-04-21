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


# TODO: Implement a motion dribbling the ball to the opponent's side.
# Issue: https://github.com/SSL-Roots/consai_ros2/issues/186

def test_our_penalty_shoot(rcst_comm: Communication):
    rcst_comm.send_empty_world()
    rcst_comm.send_ball(-2, 0)
    for i in range(11):
        rcst_comm.send_blue_robot(i, -2.5, 3.0 - i * 0.5, math.radians(0))

    rcst_comm.observer.reset()
    rcst_comm.change_referee_command('STOP', 3.0)
    rcst_comm.change_referee_command('PREPARE_PENALTY_BLUE', 3.0)
    rcst_comm.change_referee_command('NORMAL_START', 10.0)

    assert rcst_comm.observer.goal().ball_has_been_in_positive_goal() is True


def test_their_penalty_defend(rcst_comm: Communication):
    rcst_comm.send_empty_world()
    rcst_comm.send_ball(2, 0)
    rcst_comm.send_blue_robot(0, -5.5, 0.0, math.radians(0))
    rcst_comm.send_yellow_robot(0, 2.1, 0.0, math.radians(180))

    rcst_comm.observer.reset()
    rcst_comm.change_referee_command('STOP', 3.0)
    rcst_comm.change_referee_command('PREPARE_PENALTY_YELLOW', 3.0)
    rcst_comm.change_referee_command('NORMAL_START', 1.0)

    # Shoot to our goal.
    rcst_comm.send_ball(0, 0, -6.0, 0.5)
    time.sleep(5)

    assert rcst_comm.observer.goal().ball_has_been_in_negative_goal() is False


def test_robots_be_behind_ball(rcst_comm: Communication):
    rcst_comm.send_empty_world()
    for i in range(11):
        rcst_comm.send_blue_robot(i, 2.5, 3.0 - i * 0.5, math.radians(0))

    # Kicker robot.
    rcst_comm.send_blue_robot(1, -2.2, 0.0, math.radians(0))
    rcst_comm.send_ball(-2, 0)

    rcst_comm.change_referee_command('STOP', 1.0)
    rcst_comm.change_referee_command('PREPARE_PENALTY_BLUE', 0.0)

    def blue_robot_are_behind_ball(
            ball: Ball, blue_robots: RobotDict, yellow_robots: RobotDict) -> bool:
        DISTANCE = 1.0  # meters
        KICKER_ID = 1

        for robot in blue_robots.values():
            if robot.id == KICKER_ID:
                continue
            if robot.x > ball.x - DISTANCE:
                return False
        return True

    rcst_comm.observer.customized().register_sticky_true_callback(
        "test", blue_robot_are_behind_ball)

    success = False
    for _ in range(10):
        if rcst_comm.observer.customized().get_result("test"):
            success = True
            break
        time.sleep(1)
    assert success is True


def test_robots_be_behind_ball_for_their_penalty(rcst_comm: Communication):
    rcst_comm.send_empty_world()
    for i in range(11):
        rcst_comm.send_blue_robot(i, 1.0, 3.0 - i * 0.5, math.radians(0))

    rcst_comm.send_ball(2, 0)

    rcst_comm.change_referee_command('STOP', 3.0)
    rcst_comm.change_referee_command('PREPARE_PENALTY_YELLOW', 0.0)

    def blue_robot_are_behind_ball(
            ball: Ball, blue_robots: RobotDict, yellow_robots: RobotDict) -> bool:
        DISTANCE = 1.0  # meters
        GOALIE_ID = 0

        for robot in blue_robots.values():
            if robot.id == GOALIE_ID:
                continue

            if robot.x < ball.x + DISTANCE:
                return False
        return True

    rcst_comm.observer.customized().register_sticky_true_callback(
        "test", blue_robot_are_behind_ball)

    success = False
    for _ in range(10):
        if rcst_comm.observer.customized().get_result("test"):
            success = True
            break
        time.sleep(1)
    assert success is True


def test_goalie_on_the_goal_line(rcst_comm: Communication):
    rcst_comm.send_empty_world()
    rcst_comm.send_blue_robot(0, -4, 0.0, math.radians(0))

    rcst_comm.send_ball(2, 0)

    rcst_comm.change_referee_command('STOP', 3.0)
    rcst_comm.change_referee_command('PREPARE_PENALTY_YELLOW', 0.0)

    def goalie_on_the_goal_line(
            ball: Ball, blue_robots: RobotDict, yellow_robots: RobotDict) -> bool:
        GOALIE_ID = 0
        HALF_FIELD_LENGTH = -6.0  # meters
        HALF_GOAL_WIDTH = 0.9  # meters
        ROBOT_RADIUS = 0.09  # meters

        for robot in blue_robots.values():
            if robot.id != GOALIE_ID:
                continue

            if abs(robot.x - HALF_FIELD_LENGTH) < ROBOT_RADIUS and \
                    abs(robot.y) < HALF_GOAL_WIDTH:
                return True
        return False

    rcst_comm.observer.customized().register_sticky_true_callback(
        "test", goalie_on_the_goal_line)

    success = False
    for _ in range(10):
        if rcst_comm.observer.customized().get_result("test"):
            success = True
            break
        time.sleep(1)
    assert success is True
