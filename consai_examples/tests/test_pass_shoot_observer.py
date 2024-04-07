#!/usr/bin/env python3
# coding: UTF-8

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

from consai_examples.observer.pass_shoot_observer import PassShootObserver


def test_パサーロボットがいないときには空リストを返す():
    observer = PassShootObserver()
    my_robot_id = 0
    receivers_list = observer.search_receivers_list(my_robot_id)
    assert receivers_list == []


# @pytest.mark.parametrize("is_yellow, expect_can_shoot_id_list", [(False, 3), (True, 0)])
# def test_パサー以外がいないときにget_receiver_robots_idは空リストを返す(
#         rclpy_init_shutdown, is_yellow, expect_can_shoot_id_list):
#     observer = FieldObserver(our_team_is_yellow=is_yellow)
#     frame_publisher = TrackedFramePublisher()
#     frame_publisher.publish_valid_robots(blue_ids=[3])

#     # トピックをsubscribeするためspine_once()を実行
#     rclpy.spin_once(observer, timeout_sec=1.0)

#     actual_can_pass_id_list, _, actual_can_shoot_id_list, _ = observer.get_open_path_id_list(
#         my_robot_id=3)
#     assert actual_can_pass_id_list == []
#     assert len(actual_can_shoot_id_list) == expect_can_shoot_id_list


# @pytest.mark.parametrize("is_yellow", [(False), (True)])
# def test_パサー以外に敵ロボットしかいないときにget_receiver_robots_idは空リストを返す(
#         rclpy_init_shutdown, is_yellow):
#     observer = FieldObserver(our_team_is_yellow=is_yellow)
#     frame_publisher = TrackedFramePublisher()
#     frame_publisher.publish_valid_robots(blue_ids=[3], yellow_ids=[0, 1, 2, 3])

#     # トピックをsubscribeするためspine_once()を実行
#     rclpy.spin_once(observer, timeout_sec=1.0)

#     actual_can_pass_id_list, _, actual_can_shoot_id_list, _ = observer.get_open_path_id_list(
#         my_robot_id=3)
#     assert actual_can_pass_id_list == []
#     assert len(actual_can_shoot_id_list) == 3


# @pytest.mark.parametrize("is_yellow", [(False), (True)])
# def test_パサーより後ろにロボットがいるときget_receiver_robots_idは空リストを返す(
#         rclpy_init_shutdown, is_yellow):
#     observer = FieldObserver(our_team_is_yellow=is_yellow)
#     frame_publisher = TrackedFramePublisher()
#     # 右を向いたパサーの左側にロボットを置く
#     frame_publisher.set_robot(is_yellow, 3, 0.0, 0.0, 0.0)
#     frame_publisher.set_robot(is_yellow, 1, -2.0, 0.0, math.radians(30))
#     frame_publisher.set_robot(is_yellow, 4, -4.0, 0.0, math.radians(-120))
#     frame_publisher.publish_preset_frame()
#     # トピックをsubscribeするためspine_once()を実行
#     rclpy.spin_once(observer, timeout_sec=1.0)
#     actual_can_pass_id_list, \
#         actual_can_pass_pos_list, \
#         actual_can_shoot_id_list, \
#         actual_can_shoot_pos_list = observer.get_open_path_id_list(my_robot_id=3)
#     assert actual_can_pass_id_list == []
#     assert actual_can_pass_pos_list == []
#     assert len(actual_can_shoot_id_list) == 3
#     assert actual_can_shoot_pos_list[0].x == 6.0
#     assert actual_can_shoot_pos_list[0].y == 0.0

#     # 左を向いたパサーの右側にロボットを置く
#     # TODO: 後ろ向きのパス相手検索に対応できたらコメントを解除する
#     # frame_publisher.set_robot(is_yellow, 3, 0.0, 0.0, math.radians(180))
#     # frame_publisher.set_robot(is_yellow, 1, 2.0, 0.0, math.radians(30))
#     # frame_publisher.set_robot(is_yellow, 4, 4.0, 0.0, math.radians(-120))
#     # frame_publisher.publish_preset_frame()
#     # # トピックをsubscribeするためspine_once()を実行
#     # rclpy.spin_once(observer, timeout_sec=1.0)
#     # assert observer.get_receiver_robots_id(my_robot_id=3) == []


# @pytest.mark.parametrize("is_yellow", [(False), (True)])
# def test_パサーより前に味方ロボットだけがいるときにidリストを返す(
#         rclpy_init_shutdown, is_yellow):
#     observer = FieldObserver(our_team_is_yellow=is_yellow)
#     frame_publisher = TrackedFramePublisher()
#     # 右を向いたパサーの左右にロボットを置く
#     frame_publisher.set_robot(is_yellow, 3, 0.0, 0.0, 0.0)
#     frame_publisher.set_robot(is_yellow, 0, -1.0, 0.0, math.radians(0))
#     frame_publisher.set_robot(is_yellow, 1, 2.0, 0.0, math.radians(30))
#     frame_publisher.set_robot(is_yellow, 4, 4.0, 0.0, math.radians(-120))
#     frame_publisher.publish_preset_frame()
#     # トピックをsubscribeするためspine_once()を実行
#     rclpy.spin_once(observer, timeout_sec=1.0)
#     actual_can_pass_id_list, _, actual_can_shoot_id_list, _ = observer.get_open_path_id_list(
#         my_robot_id=3)
#     assert actual_can_pass_id_list == [1, 4]
#     assert len(actual_can_shoot_id_list) == 3


# @pytest.mark.parametrize("is_yellow", [(False), (True)])
# def test_パサーと味方ロボットの間に敵ロボットがいるときにidリストを返す(
#         rclpy_init_shutdown, is_yellow):
#     observer = FieldObserver(our_team_is_yellow=is_yellow)
#     frame_publisher = TrackedFramePublisher()
#     # 右を向いたパサーの右にロボットを置く
#     frame_publisher.set_robot(is_yellow, 3, 0.0, 0.0, 0.0)
#     frame_publisher.set_robot(is_yellow, 1, 2.0, 0.0, math.radians(30))
#     frame_publisher.set_robot(is_yellow, 4, 4.0, 4.0, math.radians(-120))
#     # パサーと味方ロボットの間に敵ロボットを置く
#     frame_publisher.set_robot(not is_yellow, 0, 1.0, 0.0, math.radians(0))
#     # 関係ない位置に敵ロボットを置く
#     frame_publisher.set_robot(not is_yellow, 1, 2.0, -2.0, math.radians(0))

#     frame_publisher.publish_preset_frame()
#     # トピックをsubscribeするためspine_once()を実行
#     rclpy.spin_once(observer, timeout_sec=1.0)
#     actual_can_pass_id_list, _, actual_can_shoot_id_list, _ = observer.get_open_path_id_list(
#         my_robot_id=3)
#     assert actual_can_pass_id_list == [4]
#     assert actual_can_shoot_id_list == []


# @pytest.mark.parametrize("is_yellow", [(False), (True)])
# def test_ID15付近のロボットでもget_open_path_id_listが正常動作すること(
#         rclpy_init_shutdown, is_yellow):
#     observer = FieldObserver(our_team_is_yellow=is_yellow)
#     frame_publisher = TrackedFramePublisher()
#     # 右を向いたパサーの右にロボットを置く
#     frame_publisher.set_robot(is_yellow, 15, 0.0, 0.0, 0.0)
#     frame_publisher.set_robot(is_yellow, 14, 2.0, 0.0, math.radians(30))
#     frame_publisher.set_robot(is_yellow, 13, 4.0, 4.0, math.radians(-120))
#     # パサーと味方ロボットの間に敵ロボットを置く
#     frame_publisher.set_robot(not is_yellow, 12, 1.0, 0.0, math.radians(0))
#     # 関係ない位置に敵ロボットを置く
#     frame_publisher.set_robot(not is_yellow, 11, 2.0, -2.0, math.radians(0))

#     frame_publisher.publish_preset_frame()
#     # トピックをsubscribeするためspine_once()を実行
#     rclpy.spin_once(observer, timeout_sec=1.0)
#     actual_can_pass_id_list, _, actual_can_shoot_id_list, _ = observer.get_open_path_id_list(
#         my_robot_id=15)
#     assert actual_can_pass_id_list == [13]
#     assert actual_can_shoot_id_list == []
