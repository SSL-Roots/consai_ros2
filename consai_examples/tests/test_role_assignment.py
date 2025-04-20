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

"""RoleAssignmentのテストを行うモジュール."""

from consai_examples.role_assignment import RoleAssignment, RoleName

import pytest

import rclpy

from tracked_frame_publisher import TrackedFramePublisher


@pytest.fixture
def rclpy_init_shutdown():
    """rclpyの初期化とシャットダウンを管理する関数."""
    print("rclpy.init()")
    rclpy.init()
    yield
    print("rclpy.shutdown()")
    rclpy.shutdown()


@pytest.mark.parametrize("goalie_id", [0, 1, 9, 10])
def test_引数のgoalie_idが正しく設定されること(rclpy_init_shutdown, goalie_id):
    """RoleAssignmentのgoalie_idが正しく設定されることをテストする関数."""
    assignor = RoleAssignment(goalie_id)  # 先頭でインスタンスを作成しないとエラーがでる
    frame_publisher = TrackedFramePublisher()
    frame_publisher.publish_valid_robots(blue_ids=list(range(11)))

    # トピックをsubscribeするためspine_once()を実行
    rclpy.spin_once(assignor, timeout_sec=1.0)
    assignor.update_role()
    assert assignor.get_role_from_robot_id(goalie_id) == RoleName.GOALIE


@pytest.mark.parametrize("goalie_id, is_yellow", [(1, False), (3, True)])
def test_引数のour_team_is_yellowが正しく設定されること(rclpy_init_shutdown, goalie_id, is_yellow):
    """RoleAssignmentのour_team_is_yellowが正しく設定されることをテストする関数."""
    assignor = RoleAssignment(goalie_id=goalie_id, our_team_is_yellow=is_yellow)
    frame_publisher = TrackedFramePublisher()
    frame_publisher.publish_valid_robots(blue_ids=[1], yellow_ids=[3])

    # トピックをsubscribeするためspine_once()を実行
    rclpy.spin_once(assignor, timeout_sec=1.0)
    assignor.update_role()
    assert assignor.get_role_from_robot_id(goalie_id) == RoleName.GOALIE


def test_TrackedFrameを受信するまではロールは更新されないこと(rclpy_init_shutdown):
    """TrackedFrameを受信するまでロールは更新されないことをテストする関数."""
    assignor = RoleAssignment(0)
    assignor.update_role()

    assert assignor.get_assigned_roles_and_ids() == []


def test_update_roleは更新されたロボットのIDを返すこと(rclpy_init_shutdown):
    """update_roleが更新されたロボットのIDを返すことをテストする関数."""
    assignor = RoleAssignment(0)
    frame_publisher = TrackedFramePublisher()
    frame_publisher.publish_valid_robots(blue_ids=[])

    # トピックをsubscribeするためspine_once()を実行
    rclpy.spin_once(assignor, timeout_sec=1.0)
    assert assignor.update_role() == []

    # ロボットを登場させる
    frame_publisher.publish_valid_robots(blue_ids=[0, 1, 10])
    rclpy.spin_once(assignor, timeout_sec=1.0)
    assert assignor.update_role() == [0, 1, 10]

    # ロボットを減らす
    # 誰も役割は変わらないので、空リストが返ってくる
    frame_publisher.publish_valid_robots(blue_ids=[1])
    rclpy.spin_once(assignor, timeout_sec=1.0)
    assert assignor.update_role() == []

    # ロボットを増やす
    frame_publisher.publish_valid_robots(blue_ids=[0, 1, 2, 3, 4, 5])
    rclpy.spin_once(assignor, timeout_sec=1.0)
    assert assignor.update_role() == [0, 2, 3, 4, 5]

    # ロボットを減らして増やす
    # 空きロールに新しいIDがセットされるの
    # ID5の担当は変わらない [0, 1, 2, 8, 9, 5]
    frame_publisher.publish_valid_robots(blue_ids=[0, 1, 2, 5, 8, 9])
    rclpy.spin_once(assignor, timeout_sec=1.0)
    assert assignor.update_role() == [8, 9]


def test_ロールの数よりロボットが多くてもエラーが発生しないこと(rclpy_init_shutdown):
    """ロールの数よりロボットが多くてもエラーが発生しないことをテストする関数."""
    assignor = RoleAssignment(0)
    frame_publisher = TrackedFramePublisher()
    # 12台のロボットを用意する
    frame_publisher.publish_valid_robots(blue_ids=list(range(13)))

    # トピックをsubscribeするためspine_once()を実行
    rclpy.spin_once(assignor, timeout_sec=1.0)
    assignor.update_role()
    assert len(assignor.get_assigned_roles_and_ids()) == 11


def test_ロボットが消えても優先度の高いロールは空けないこと(rclpy_init_shutdown):
    """ロボットが消えても優先度の高いロールは空けないことをテストする関数."""
    assignor = RoleAssignment(0)
    frame_publisher = TrackedFramePublisher()
    frame_publisher.publish_valid_robots(blue_ids=[0, 3, 4, 5, 6, 7])

    rclpy.spin_once(assignor, timeout_sec=1.0)
    assignor.update_role()

    # 4番と5番を退場させる
    frame_publisher.publish_valid_robots(blue_ids=[0, 3, 6, 7])
    rclpy.spin_once(assignor, timeout_sec=1.0)
    changed_ids = assignor.update_role()
    assert changed_ids == [7, 6]
    assert assignor.get_assigned_roles_and_ids() == [
        (RoleName.GOALIE, 0),
        (RoleName.ATTACKER, 3),
        (RoleName.SUB_ATTACKER, 7),
        (RoleName.CENTER_BACK1, 6),
    ]


def test_ボールに一番近いロボットがAttackerになること(rclpy_init_shutdown):
    """ボールに一番近いロボットがAttackerになることをテストする関数."""
    assignor = RoleAssignment(0)
    frame_publisher = TrackedFramePublisher()
    # ID9のロボットが一番ボールに近い
    frame_publisher.set_robot_pos(False, 7, -5.0, 0.0)
    frame_publisher.set_robot_pos(False, 8, -2.0, 0.0)
    frame_publisher.set_robot_pos(False, 9, 4.0, 0.0)
    frame_publisher.set_ball_pos(3.0, 0.0)
    frame_publisher.publish_preset_frame()

    rclpy.spin_once(assignor, timeout_sec=1.0)
    assignor.update_role()

    assert assignor.get_assigned_roles_and_ids() == [
        (RoleName.ATTACKER, 9),
        (RoleName.SUB_ATTACKER, 8),
        (RoleName.CENTER_BACK1, 7),
    ]


def test_goalieが一番ボールに近いときは二番目に近いロボットがAttackerになること(rclpy_init_shutdown):
    """Goalieが一番ボールに近い場合, 二番目に近いロボットがAttackerになることをテストする関数."""
    assignor = RoleAssignment(9)
    frame_publisher = TrackedFramePublisher()
    # ID9のロボットが一番ボールに近い、がGoalieである
    frame_publisher.set_robot_pos(False, 7, -5.0, 0.0)
    frame_publisher.set_robot_pos(False, 8, -2.0, 0.0)
    frame_publisher.set_robot_pos(False, 9, 4.0, 0.0)
    frame_publisher.set_ball_pos(3.0, 0.0)
    frame_publisher.publish_preset_frame()

    rclpy.spin_once(assignor, timeout_sec=1.0)
    assignor.update_role()

    assert assignor.get_assigned_roles_and_ids() == [
        (RoleName.GOALIE, 9),
        (RoleName.ATTACKER, 8),
        (RoleName.SUB_ATTACKER, 7),
    ]


def test_ボール位置によってAttackerを更新しないフラグが適用されること(rclpy_init_shutdown):
    """ボール位置によってAttackerを更新しないフラグが適用されることをテストする関数."""
    assignor = RoleAssignment(0)
    frame_publisher = TrackedFramePublisher()
    # ボールに一番近いのはID9だが、IDが一番小さい7がAttackerとなる
    frame_publisher.set_robot_pos(False, 7, -5.0, 0.0)
    frame_publisher.set_robot_pos(False, 8, -2.0, 0.0)
    frame_publisher.set_robot_pos(False, 9, 4.0, 0.0)
    frame_publisher.set_ball_pos(3.0, 0.0)
    frame_publisher.publish_preset_frame()

    rclpy.spin_once(assignor, timeout_sec=1.0)
    assignor.update_role(update_attacker_by_ball_pos=False)

    assert assignor.get_assigned_roles_and_ids() == [
        (RoleName.ATTACKER, 7),
        (RoleName.SUB_ATTACKER, 8),
        (RoleName.CENTER_BACK1, 9),
    ]


@pytest.mark.parametrize("robot_num, expected_indexes", [(11, []), (10, [10]), (9, [10, 9]), (8, [10, 9, 8])])
def test_ロボットの出場可能台数が減ったら優先度の低い順にSUBSTITUEロールを割り当てること(rclpy_init_shutdown, robot_num, expected_indexes):
    """ロボットの出場可能台数が減った場合に, 優先度の低い順にSUBSTITUTEロールを割り当てることをテストする関数."""
    assignor = RoleAssignment(0)
    frame_publisher = TrackedFramePublisher()
    frame_publisher.publish_valid_robots(blue_ids=list(range(11)))

    # トピックをsubscribeするためspine_once()を実行
    rclpy.spin_once(assignor, timeout_sec=1.0)
    assignor.update_role(allowed_robot_num=robot_num)
    for expected_index in expected_indexes:
        assert assignor.get_role_from_robot_id(expected_index) == RoleName.SUBSTITUTE


def test_SUBSTITUTEから復帰した場合もchanged_idとして返すこと(rclpy_init_shutdown):
    """SUBSTITUTEから復帰した場合もchanged_idとして返すことをテストする関数."""
    assignor = RoleAssignment(0)
    frame_publisher = TrackedFramePublisher()
    frame_publisher.publish_valid_robots(blue_ids=list(range(11)))

    # トピックをsubscribeするためspine_once()を実行
    rclpy.spin_once(assignor, timeout_sec=1.0)
    changed_ids = assignor.update_role(allowed_robot_num=11)
    assert len(changed_ids) == 11

    # 許可台数を一つ減らす
    changed_ids = assignor.update_role(allowed_robot_num=10)
    assert len(changed_ids) == 1
    assert changed_ids[0] == 10

    # 復活させる
    changed_ids = assignor.update_role(allowed_robot_num=11)
    assert len(changed_ids) == 1
    assert changed_ids[0] == 10

    # もう一度更新する。roleの割り当てに変化はない
    changed_ids = assignor.update_role(allowed_robot_num=11)
    assert len(changed_ids) == 0
