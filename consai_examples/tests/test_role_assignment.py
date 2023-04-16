
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
    assert assignor.get_role_from_robot_id(goalie_id) == RoleName.GOALIE


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
    assert assignor.get_role_from_robot_id(goalie_id) == RoleName.GOALIE


def test_TrackedFrameを受信するまではロールは更新されないこと(rclpy_init_shutdown):
    assignor = RoleAssignment(0)
    assignor.update_role()

    assert assignor.get_assigned_roles() == []


def test_ロールの数よりロボットが多くてもエラーが発生しないこと(rclpy_init_shutdown):
    assignor = RoleAssignment(0)
    frame_publisher = TrackedFramePublisher()
    # 12台のロボットを用意する
    frame_publisher.publish_valid_robots(blue_ids=list(range(13)))

    # トピックをsubscribeするためspine_once()を実行
    rclpy.spin_once(assignor, timeout_sec=1.0)
    assignor.update_role()
    assert len(assignor.get_assigned_roles()) == 11


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
    assert changed_roles == [RoleName.CENTER_BACK1, RoleName.CENTER_BACK2]
    assert assignor.get_assigned_roles_and_ids() == [
        (RoleName.GOALIE, 0),
        (RoleName.ATTACKER, 3),
        (RoleName.CENTER_BACK1, 7),
        (RoleName.CENTER_BACK2, 6)]


def test_ボールに一番近いロボットがAttackerになること(rclpy_init_shutdown):
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
        (RoleName.CENTER_BACK1, 8),
        (RoleName.CENTER_BACK2, 7)]

def test_goalieが一番ボールに近いときは二番目に近いロボットがAttackerになること(rclpy_init_shutdown):
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
        (RoleName.CENTER_BACK1, 7)]


def test_ボール位置によってAttackerを更新しないフラグが適用されること(rclpy_init_shutdown):
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
        (RoleName.CENTER_BACK1, 8),
        (RoleName.CENTER_BACK2, 9)]
