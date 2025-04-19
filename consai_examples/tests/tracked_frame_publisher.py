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

from rclpy.node import Node

from robocup_ssl_msgs.msg import RobotId, TrackedBall, TrackedFrame, TrackedRobot, Vector2


# TrackedFrameトピックをpublishするためのクラス


class TrackedFramePublisher(Node):
    def __init__(self):
        super().__init__('publisher')

        self._publisher = self.create_publisher(TrackedFrame, 'detection_tracked', 1)
        self._preset_frame = TrackedFrame()

    def _publish(self, frame):
        self._publisher.publish(frame)
        print("Test topic published")

    def publish_empty_data(self):
        frame = TrackedFrame()
        self._publish(frame)

    def publish_valid_robots(self, blue_ids=[], yellow_ids=[]):
        frame = TrackedFrame()
        for blue_id in blue_ids:
            robot = TrackedRobot()
            robot.robot_id.id = blue_id
            robot.robot_id.team_color = RobotId.TEAM_COLOR_BLUE
            robot.visibility.append(1.0)  # ここが0だとフィールドにいない判定になる
            frame.robots.append(robot)

        for yellow_id in yellow_ids:
            robot = TrackedRobot()
            robot.robot_id.id = yellow_id
            robot.robot_id.team_color = RobotId.TEAM_COLOR_YELLOW
            robot.visibility.append(1.0)  # ここが0だとフィールドにいない判定になる
            frame.robots.append(robot)

        self._publish(frame)

    def set_robot_pos(self, is_yellow, robot_id, pos_x, pos_y):
        robot = TrackedRobot()
        robot.robot_id.id = robot_id
        robot.robot_id.team_color = RobotId.TEAM_COLOR_YELLOW
        if is_yellow is False:
            robot.robot_id.team_color = RobotId.TEAM_COLOR_BLUE
        robot.pos.x = pos_x
        robot.pos.y = pos_y
        robot.visibility.append(1.0)
        self._preset_frame.robots.append(robot)

    def set_robot(self, is_yellow, robot_id, pos_x, pos_y, orientation,
                  vel_x=0.0, vel_y=0.0, vel_angular=0.0):
        robot = TrackedRobot()
        robot.robot_id.id = robot_id
        robot.robot_id.team_color = RobotId.TEAM_COLOR_YELLOW
        if is_yellow is False:
            robot.robot_id.team_color = RobotId.TEAM_COLOR_BLUE
        robot.pos.x = pos_x
        robot.pos.y = pos_y
        robot.orientation = orientation
        robot.vel.append(Vector2(x=vel_x, y=vel_y))
        robot.vel_angular.append(vel_angular)
        robot.visibility.append(1.0)
        self._preset_frame.robots.append(robot)

    def set_ball_pos(self, pos_x, pos_y):
        ball = TrackedBall()
        ball.pos.x = pos_x
        ball.pos.y = pos_y
        ball.visibility.append(1.0)
        self._preset_frame.balls.append(ball)

    def publish_preset_frame(self):
        self._publish(self._preset_frame)

    def clear_preset_frame(self):
        self._preset_frame = TrackedFrame()
