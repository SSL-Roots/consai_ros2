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

from utils_for_placement import init_placement


def test_ボールプレースメント時に進入禁止エリアに侵入しないこと(rcst_comm: Communication):
    target_x = 0.0  # meters
    target_y = 0.0  # meters
    ball_x = -4.0  # meters
    ball_y = 0.0  # meters
    init_placement(
        rcst_comm, color="yellow", target_x=target_x, target_y=target_y,num_of_robots=11, ball_x=ball_x, ball_y=ball_y
    )
    time.sleep(3)

    def robot_is_invading_the_placement_area(
            ball: Ball, blue_robots: RobotDict, yellow_robots: RobotDict) -> bool:
        DISTANCE = 0.5  # meters
        GOALIE_ID = 0

        for robot in blue_robots.values():
            if robot.id == GOALIE_ID:
                continue

            distance = calc.distance_line_ab_to_point_c(
                target_x, target_y, ball.x, ball.y, robot.x, robot.y
            )

            if distance < DISTANCE:
                return True
        return False

    rcst_comm.observer.customized().register_sticky_true_callback(
        "test", robot_is_invading_the_placement_area)

    success = True
    for _ in range(10):
        if rcst_comm.observer.customized().get_result("test"):
            success = False
            break
        time.sleep(1)
    assert success is True


def test_インプレイ中にディフェンスエリアに侵入しないこと(rcst_comm: Communication):
    rcst_comm.send_empty_world()
    robot_num = 11
    for i in range(robot_num):
        rcst_comm.send_blue_robot(i, -1.0, 3.0 - 0.5*i, 0.0)
    rcst_comm.send_ball(4.5, 0)

    def robot_is_in_the_defense_area(
            ball: Ball, blue_robots: RobotDict, yellow_robots: RobotDict) -> bool:
        GOALIE_ID = 0
        HALF_FIELD_LENGTH = 6.0  # meters
        DEFENSE_WIDTH = 1.8  # meters
        DEFENSE_HALF_HEIGHT = 1.8  # meters

        for robot in blue_robots.values():
            if robot.id == GOALIE_ID:
                continue

            if (abs(robot.y)) < DEFENSE_HALF_HEIGHT and \
                    (abs(robot.x)) > (HALF_FIELD_LENGTH - DEFENSE_WIDTH):
                return True
        return False

    rcst_comm.observer.customized().register_sticky_true_callback(
        "test", robot_is_in_the_defense_area)

    # TODO(ゴール前ディンフェスがロボットを衝突回避して、ディフェンスエリアに侵入する)
    # TODO(アタッカーがボールを追いかけて、急激に目標位置が変わり、ディンフェスエリアに侵入する)
    success = True
    ball_positions = [
        # (0.0, 0.0),
        (4.5, 1.5),
        # (4.5, -1.5),
        # (4.5, 1.5),
        # (5.5, -1.5),
    ]

    for ball_x, ball_y in ball_positions:
        rcst_comm.change_referee_command('STOP', 3.0)
        rcst_comm.send_ball(ball_x, ball_y)
        rcst_comm.observer.reset()
        rcst_comm.change_referee_command('FORCE_START', 0.0)

        for _ in range(5):
            if rcst_comm.observer.customized().get_result("test"):
                success = False
                break
            time.sleep(1)
        assert success is True
