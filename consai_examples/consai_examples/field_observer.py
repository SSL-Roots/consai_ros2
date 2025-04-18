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

import json

from consai_examples.observer.ball_motion_observer import BallMotionObserver
from consai_examples.observer.ball_placement_observer import BallPlacementObserver
from consai_examples.observer.ball_position_observer import BallPositionObserver
from consai_examples.observer.detection_wrapper import DetectionWrapper
from consai_examples.observer.distance_observer import DistanceObserver
from consai_examples.observer.field_normalizer import FieldNormalizer
from consai_examples.observer.field_positions import FieldPositions
from consai_examples.observer.man_mark_observer import ManMarkObserver
from consai_examples.observer.pass_shoot_observer import PassShootObserver
from consai_examples.observer.side_back_target_observer import SideBackTargetObserver
from consai_examples.observer.zone_ball_observer import ZoneBallObserver
from consai_examples.observer.zone_man_mark_target_observer import ZoneManMarkTargetObserver
from consai_examples.observer.zone_target_observer import ZoneTargetObserver

from consai_msgs.msg import GoalPose, GoalPoses, State2D

from consai_visualizer_msgs.msg import Objects

from rclpy import qos
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from robocup_ssl_msgs.msg import TrackedFrame

from std_msgs.msg import String


class FieldObserver(Node):

    def __init__(self, goalie_id, our_team_is_yellow=False):
        super().__init__('field_observer')

        self._logger = self.get_logger()

        self._field_normalizer = FieldNormalizer()
        self._field_positions = FieldPositions()

        self._sub_detection_traced = self.create_subscription(
            TrackedFrame, 'detection_tracked', self._detection_tracked_callback, 10)

        self._pub_visualizer_objects = self.create_publisher(
            Objects, 'visualizer_objects', qos.qos_profile_sensor_data)

        self._sub_destinations = self.create_subscription(
            GoalPoses, 'destinations', self._destinations_callback, 10)

        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self._sub_param_rule = self.create_subscription(
            String, 'consai_param/rule', self._param_rule_callback, qos_profile)
        self._sub_param_strategy = self.create_subscription(
            String, 'consai_param/strategy', self._param_strategy_callback, qos_profile)

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

    def _param_rule_callback(self, msg):
        param_dict = json.loads(msg.data)
        self._field_normalizer.set_field_size(
            param_dict['field']['length'],
            param_dict['field']['width'],
            param_dict['field']['goal_width'],
            param_dict['field']['penalty_depth'],
            param_dict['field']['penalty_width']
        )
        self._field_normalizer.set_robot_diameter(
            param_dict['robots']['max_diameter']
        )
        self._field_normalizer.set_ball_diameter(
            param_dict['ball']['diameter']
        )

        self._field_positions.set_field_size(
            param_dict['field']['length'],
            param_dict['field']['goal_width'],
            param_dict['field']['penalty_depth'],
            param_dict['field']['penalty_width']
        )

        self._pass_shoot_observer.set_field_normalizer(self._field_normalizer)
        self._zone_ball_observer.set_field_normalizer(self._field_normalizer)
        self._ball_position_state_observer.set_field_normalizer(self._field_normalizer)
        self._man_mark_observer.set_field_positions(self._field_positions)
        self._side_back_target_observer.set_field_positions(self._field_positions)
        self._zone_man_mark_target_observer.set_field_positions(self._field_positions)

        self._logger.info('Field size is updated')

    def _param_strategy_callback(self, msg):
        param_dict = json.loads(msg.data)
        self._field_normalizer.set_div_a_size(
            param_dict['div_a_field']['length'],
            param_dict['div_a_field']['width'],
            param_dict['div_a_field']['robot_diameter'],
            param_dict['div_a_field']['ball_diameter']
        )

        self._logger.info('Div A size is updated')

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

    def destination_pose(self, robot_id: int) -> tuple[bool, State2D]:
        for goal_pose in self._destinations.poses:
            if goal_pose.robot_id == robot_id:
                return True, goal_pose.pose
        return False, GoalPose().pose

    def set_num_of_zone_roles(self, num_of_zone_roles: int) -> None:
        self._num_of_zone_roles = num_of_zone_roles

    def field_pos(self) -> FieldPositions:
        return self._field_positions

    def field_half_length(self) -> float:
        return self._field_normalizer.half_length()

    def field_half_width(self) -> float:
        return self._field_normalizer.half_width()

    def field_margin_to_wall(self) -> float:
        return 0.3

    def on_div_a_x(self, x: float) -> float:
        # X座標を正規化する
        return self._field_normalizer.on_div_a_x(x)

    def on_div_a_y(self, y: float) -> float:
        # Y座標を正規化する
        return self._field_normalizer.on_div_a_y(y)

    def on_div_a_robot_diameter(self, size: float) -> float:
        # ロボットの直径を正規化する
        return self._field_normalizer.on_div_a_robot_diameter(size)

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
