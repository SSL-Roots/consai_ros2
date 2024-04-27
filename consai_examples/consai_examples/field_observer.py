#!/usr/bin/env python3
# coding: UTF-8

# Copyright 2021 Roots
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

from rclpy import qos
from rclpy.node import Node
from robocup_ssl_msgs.msg import TrackedFrame

from consai_examples.observer.detection_wrapper import DetectionWrapper
from consai_examples.observer.ball_position_observer import BallPositionObserver
from consai_examples.observer.ball_placement_observer import BallPlacementObserver
from consai_examples.observer.side_back_target_observer import SideBackTargetObserver
from consai_examples.observer.zone_ball_observer import ZoneBallObserver
from consai_examples.observer.zone_man_mark_target_observer import ZoneManMarkTargetObserver
from consai_examples.observer.zone_target_observer import ZoneTargetObserver
from consai_examples.observer.ball_motion_observer import BallMotionObserver
from consai_examples.observer.pass_shoot_observer import PassShootObserver
from consai_examples.observer.distance_observer import DistanceObserver
from consai_examples.observer.man_mark_observer import ManMarkObserver
from consai_msgs.msg import GoalPoses, GoalPose
from consai_visualizer_msgs.msg import Objects


class FieldObserver(Node):

    def __init__(self, goalie_id, our_team_is_yellow=False):
        super().__init__('field_observer')

        self._field_length = 12.0  # meters
        self._field_half_length = self._field_length * 0.5
        self._field_width = 9.0  # meters
        self._field_half_width = self._field_width * 0.5

        self._sub_detection_traced = self.create_subscription(
            TrackedFrame, 'detection_tracked', self._detection_tracked_callback, 10)

        self._pub_visualizer_objects = self.create_publisher(
            Objects, 'visualizer_objects', qos.qos_profile_sensor_data)

        self._sub_destinations = self.create_subscription(
            GoalPoses, 'destinations', self._destinations_callback, 10)

        self._detection_wrapper = DetectionWrapper(our_team_is_yellow)
        self._ball_position_state_observer = BallPositionObserver()
        self._ball_placement_observer = BallPlacementObserver()
        self._zone_ball_observer = ZoneBallObserver()
        self._zone_target_observer = ZoneTargetObserver()
        self._side_back_target_observer = SideBackTargetObserver()
        self._zone_man_mark_target_observer = ZoneManMarkTargetObserver()
        self._ball_motion_observer = BallMotionObserver()
        self._pass_shoot_observer = PassShootObserver(goalie_id)
        self._distance_observer = DistanceObserver()
        self._man_mark_observer = ManMarkObserver()

        self._destinations = GoalPoses()

        self._num_of_zone_roles = 0

    def detection(self) -> DetectionWrapper:
        return self._detection_wrapper

    def ball_position(self) -> BallPositionObserver:
        return self._ball_position_state_observer

    def ball_placement(self) -> BallPlacementObserver:
        return self._ball_placement_observer

    def zone(self) -> ZoneBallObserver:
        return self._zone_ball_observer

    def zone_target(self) -> ZoneTargetObserver:
        return self._zone_target_observer

    def side_back_target(self) -> SideBackTargetObserver:
        return self._side_back_target_observer

    def zone_man_mark_target(self) -> ZoneManMarkTargetObserver:
        return self._zone_man_mark_target_observer

    def ball_motion(self) -> BallMotionObserver:
        return self._ball_motion_observer

    def pass_shoot(self) -> PassShootObserver:
        return self._pass_shoot_observer

    def distance(self) -> DistanceObserver:
        return self._distance_observer

    def man_mark(self) -> ManMarkObserver:
        return self._man_mark_observer

    def destination(self, robot_id: int) -> tuple[bool, GoalPose]:
        for goal_pose in self._destinations.poses:
            if goal_pose.robot_id == robot_id:
                return True, goal_pose
        return False, GoalPose()

    def set_num_of_zone_roles(self, num_of_zone_roles: int) -> None:
        self._num_of_zone_roles = num_of_zone_roles

    def field_half_length(self) -> float:
        return self._field_half_length

    def field_half_width(self) -> float:
        return self._field_half_width

    def field_margin_to_wall(self) -> float:
        return 0.3

    def _detection_tracked_callback(self, msg):
        self._detection_wrapper.update(msg)

        self._ball_position_state_observer.update(
            self._detection_wrapper.ball().pos())
        self._ball_placement_observer.update(self._detection_wrapper.ball())
        self._zone_ball_observer.update(
            self._detection_wrapper.ball().pos(),
            self.ball_position().is_in_our_side())
        self._zone_target_observer.update(
            self._detection_wrapper.their_robots(), self._num_of_zone_roles)
        self._side_back_target_observer.update(self._detection_wrapper.their_robots())
        self._zone_man_mark_target_observer.update(self._detection_wrapper.their_robots())
        self._ball_motion_observer.update(self._detection_wrapper.ball())
        self._pass_shoot_observer.update(
            self._detection_wrapper.ball(),
            self._detection_wrapper.our_robots(),
            self._detection_wrapper.their_robots())
        self._distance_observer.update(
            self._detection_wrapper.ball().pos(),
            self._detection_wrapper.our_robots(),
            self._detection_wrapper.their_robots())
        self._man_mark_observer.update(
            self._detection_wrapper.ball(),
            self._detection_wrapper.our_robots(),
            self._detection_wrapper.their_robots())

        self._publish_visualizer_msgs()

    def _destinations_callback(self, msg):
        self._destinations = msg

    def _publish_visualizer_msgs(self):
        self._pub_visualizer_objects.publish(
            self._man_mark_observer.to_visualize_msg(
                self._detection_wrapper.our_robots(),
                self._detection_wrapper.their_robots()))
