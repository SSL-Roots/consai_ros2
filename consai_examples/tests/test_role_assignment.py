
from consai_examples.role_assignment import RoleAssignment
from consai_examples.role_assignment import RoleName
import pytest
import rclpy
from tracked_frame_publisher import TrackedFramePublisher


@pytest.fixture
def rclpy_init_shutdown():
    print("rclpy.init()")
    rclpy.init()
    yield
    print("rclpy.shutdown()")
    rclpy.shutdown()


@pytest.mark.parametrize("goalie_id", [0, 1, 9, 10])
def test_引数のgoalie_idが正しく設定されること(rclpy_init_shutdown, goalie_id):
    assignor = RoleAssignment(goalie_id)  # 先頭でインスタンスを作成しないとエラーがでる
    frame_publisher = TrackedFramePublisher()
    frame_publisher.publish_valid_robots(blue_ids=list(range(11)))

    # トピックをsubscribeするためspine_once()を実行
    rclpy.spin_once(assignor, timeout_sec=1.0)
    assignor.update_role()
    assert assignor.get_robot_id(RoleName.GOALIE.value) == goalie_id


@pytest.mark.parametrize("goalie_id, is_yellow", [(1, False), (3, True)])
def test_引数のour_team_is_yellowが正しく設定されること(rclpy_init_shutdown, goalie_id, is_yellow):
    assignor = RoleAssignment(
        goalie_id=goalie_id,
        our_team_is_yellow=is_yellow)
    frame_publisher = TrackedFramePublisher()
    frame_publisher.publish_valid_robots(blue_ids=[1], yellow_ids=[3])

    # トピックをsubscribeするためspine_once()を実行
    rclpy.spin_once(assignor, timeout_sec=1.0)
    assignor.update_role()
    assert assignor.get_robot_id(RoleName.GOALIE.value) == goalie_id


def test_TrackedFrameを受信するまではロールは更新されないこと(rclpy_init_shutdown):
    assignor = RoleAssignment(0)
    assignor.update_role()

    active_roles = assignor.get_active_roles()
    assert active_roles == []


def test_ロールの数よりロボットが多くてもエラーが発生しないこと(rclpy_init_shutdown):
    assignor = RoleAssignment(0)
    frame_publisher = TrackedFramePublisher()
    # 12台のロボットを用意する
    frame_publisher.publish_valid_robots(blue_ids=list(range(13)))

    # トピックをsubscribeするためspine_once()を実行
    rclpy.spin_once(assignor, timeout_sec=1.0)
    assignor.update_role()
    active_roles = assignor.get_active_roles()
    assert len(active_roles) == 11


def test_ロボットが消えても優先度の高いロールは空けないこと(rclpy_init_shutdown):
    assignor = RoleAssignment(0)
    frame_publisher = TrackedFramePublisher()
    frame_publisher.publish_valid_robots(blue_ids=[0,3,4,5,6,7])

    rclpy.spin_once(assignor, timeout_sec=1.0)
    assignor.update_role()

    # 4番と5番を退場させる
    frame_publisher.publish_valid_robots(blue_ids=[0,3,6,7])
    rclpy.spin_once(assignor, timeout_sec=1.0)
    changed_roles = assignor.update_role()
    assert changed_roles == [2, 3]
    
    active_roles = assignor.get_active_roles()
    assert active_roles == [0, 1, 2, 3]


def test_ボールに一番近いロボットがAttackerになること(rclpy_init_shutdown):
    assignor = RoleAssignment(0)
    frame_publisher = TrackedFramePublisher()
    frame_publisher.set_robot_pos(False, 7, -5.0, 0.0)
    frame_publisher.set_robot_pos(False, 8, -2.0, 0.0)
    frame_publisher.set_robot_pos(False, 9, -1.0, 0.0)
    frame_publisher.set_ball_pos(-2.0, -0.0)
    frame_publisher.publish_preset_frame()

    rclpy.spin_once(assignor, timeout_sec=1.0)
    assignor.update_role()

    active_roles = assignor.get_active_roles()
    assert active_roles == [RoleName.ATTACKER.value, 2, 3]
    assert assignor.get_robot_id(RoleName.ATTACKER.value) == 8

def test_ボール位置によってAttackerを更新しないフラグが適用されること(rclpy_init_shutdown):
    assignor = RoleAssignment(0)
    frame_publisher = TrackedFramePublisher()
    # ボールに一番近いのはID9だが、IDが一番小さい7がAttackerとなる
    frame_publisher.set_robot_pos(False, 7, -5.0, 0.0)
    frame_publisher.set_robot_pos(False, 8, -2.0, 0.0)
    frame_publisher.set_robot_pos(False, 9, -1.0, 0.0)
    frame_publisher.set_ball_pos(-2.0, -0.0)
    frame_publisher.publish_preset_frame()

    rclpy.spin_once(assignor, timeout_sec=1.0)
    assignor.update_role(update_attacker_by_ball_pos=False)

    active_roles = assignor.get_active_roles()
    assert active_roles == [RoleName.ATTACKER.value, 2, 3]
    assert assignor.get_robot_id(RoleName.ATTACKER.value) == 7

