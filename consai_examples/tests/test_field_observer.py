
from consai_examples.field_observer import FieldObserver
import math
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


@pytest.mark.parametrize("is_yellow", [(False), (True)])
def test_パサーロボットがいないときにget_receiver_robots_idは空リストを返す(rclpy_init_shutdown, is_yellow):
    observer = FieldObserver(our_team_is_yellow=is_yellow)
    actual_can_pass_id_list, actual_can_shoot_id_list = observer.get_open_path_id_list(my_robot_id=3)
    assert actual_can_pass_id_list == []
    assert len(actual_can_shoot_id_list) == 0


@pytest.mark.parametrize("is_yellow, expect_can_shoot_id_list", [(False, 5), (True, 0)])
def test_パサー以外がいないときにget_receiver_robots_idは空リストを返す(rclpy_init_shutdown, is_yellow, expect_can_shoot_id_list):
    observer = FieldObserver(our_team_is_yellow=is_yellow)
    frame_publisher = TrackedFramePublisher()
    frame_publisher.publish_valid_robots(blue_ids=[3])

    # トピックをsubscribeするためspine_once()を実行
    rclpy.spin_once(observer, timeout_sec=1.0)

    actual_can_pass_id_list, actual_can_shoot_id_list = observer.get_open_path_id_list(my_robot_id=3)
    assert actual_can_pass_id_list == []
    assert len(actual_can_shoot_id_list) == expect_can_shoot_id_list

@pytest.mark.parametrize("is_yellow", [(False), (True)])
def test_パサー以外に敵ロボットしかいないときにget_receiver_robots_idは空リストを返す(rclpy_init_shutdown, is_yellow):
    observer = FieldObserver(our_team_is_yellow=is_yellow)
    frame_publisher = TrackedFramePublisher()
    frame_publisher.publish_valid_robots(blue_ids=[3], yellow_ids=[0, 1, 2, 3])

    # トピックをsubscribeするためspine_once()を実行
    rclpy.spin_once(observer, timeout_sec=1.0)

    actual_can_pass_id_list, actual_can_shoot_id_list = observer.get_open_path_id_list(my_robot_id=3)
    assert actual_can_pass_id_list == []
    assert len(actual_can_shoot_id_list) == 5

@pytest.mark.parametrize("is_yellow", [(False), (True)])
def test_パサーより後ろにロボットがいるときget_receiver_robots_idは空リストを返す(rclpy_init_shutdown, is_yellow):
    observer = FieldObserver(our_team_is_yellow=is_yellow)
    frame_publisher = TrackedFramePublisher()
    # 右を向いたパサーの左側にロボットを置く
    frame_publisher.set_robot(is_yellow, 3, 0.0, 0.0, 0.0)
    frame_publisher.set_robot(is_yellow, 1, -2.0, 0.0, math.radians(30))
    frame_publisher.set_robot(is_yellow, 4, -4.0, 0.0, math.radians(-120))
    frame_publisher.publish_preset_frame()
    # トピックをsubscribeするためspine_once()を実行
    rclpy.spin_once(observer, timeout_sec=1.0)
    actual_can_pass_id_list, actual_can_shoot_id_list = observer.get_open_path_id_list(my_robot_id=3)
    assert actual_can_pass_id_list == []
    assert len(actual_can_shoot_id_list) == 5

    # 左を向いたパサーの右側にロボットを置く
    # TODO: 後ろ向きのパス相手検索に対応できたらコメントを解除する
    # frame_publisher.set_robot(is_yellow, 3, 0.0, 0.0, math.radians(180))
    # frame_publisher.set_robot(is_yellow, 1, 2.0, 0.0, math.radians(30))
    # frame_publisher.set_robot(is_yellow, 4, 4.0, 0.0, math.radians(-120))
    # frame_publisher.publish_preset_frame()
    # # トピックをsubscribeするためspine_once()を実行
    # rclpy.spin_once(observer, timeout_sec=1.0)
    # assert observer.get_receiver_robots_id(my_robot_id=3) == []

@pytest.mark.parametrize("is_yellow", [(False), (True)])
def test_パサーより前に味方ロボットだけがいるときget_receiver_robots_idがidリストを返す(rclpy_init_shutdown, is_yellow):
    observer = FieldObserver(our_team_is_yellow=is_yellow)
    frame_publisher = TrackedFramePublisher()
    # 右を向いたパサーの左右にロボットを置く
    frame_publisher.set_robot(is_yellow, 3, 0.0, 0.0, 0.0)
    frame_publisher.set_robot(is_yellow, 0, -1.0, 0.0, math.radians(0))
    frame_publisher.set_robot(is_yellow, 1, 2.0, 0.0, math.radians(30))
    frame_publisher.set_robot(is_yellow, 4, 4.0, 0.0, math.radians(-120))
    frame_publisher.publish_preset_frame()
    # トピックをsubscribeするためspine_once()を実行
    rclpy.spin_once(observer, timeout_sec=1.0)
    actual_can_pass_id_list, actual_can_shoot_id_list = observer.get_open_path_id_list(my_robot_id=3)
    assert actual_can_pass_id_list == [1, 4]
    assert len(actual_can_shoot_id_list) == 5

@pytest.mark.parametrize("is_yellow", [(False), (True)])
def test_パサーと味方ロボットの間に敵ロボットがいるときget_receiver_robots_idがパスできるidリストを返す(rclpy_init_shutdown, is_yellow):
    observer = FieldObserver(our_team_is_yellow=is_yellow)
    frame_publisher = TrackedFramePublisher()
    # 右を向いたパサーの右にロボットを置く
    frame_publisher.set_robot(is_yellow, 3, 0.0, 0.0, 0.0)
    frame_publisher.set_robot(is_yellow, 1, 2.0, 0.0, math.radians(30))
    frame_publisher.set_robot(is_yellow, 4, 4.0, 4.0, math.radians(-120))
    # パサーと味方ロボットの間に敵ロボットを置く
    frame_publisher.set_robot(not is_yellow, 0, 1.0, 0.0, math.radians(0))
    # 関係ない位置に敵ロボットを置く
    frame_publisher.set_robot(not is_yellow, 1, 2.0, -2.0, math.radians(0))

    frame_publisher.publish_preset_frame()
    # トピックをsubscribeするためspine_once()を実行
    rclpy.spin_once(observer, timeout_sec=1.0)
    actual_can_pass_id_list, actual_can_shoot_id_list = observer.get_open_path_id_list(my_robot_id=3)
    assert actual_can_pass_id_list == [4]
    assert actual_can_shoot_id_list == []

@pytest.mark.parametrize("is_yellow", [(False), (True)])
def test_ID15付近のロボットでもget_receiver_robots_idが正常動作すること(rclpy_init_shutdown, is_yellow):
    observer = FieldObserver(our_team_is_yellow=is_yellow)
    frame_publisher = TrackedFramePublisher()
    # 右を向いたパサーの右にロボットを置く
    frame_publisher.set_robot(is_yellow, 15, 0.0, 0.0, 0.0)
    frame_publisher.set_robot(is_yellow, 14, 2.0, 0.0, math.radians(30))
    frame_publisher.set_robot(is_yellow, 13, 4.0, 4.0, math.radians(-120))
    # パサーと味方ロボットの間に敵ロボットを置く
    frame_publisher.set_robot(not is_yellow, 12, 1.0, 0.0, math.radians(0))
    # 関係ない位置に敵ロボットを置く
    frame_publisher.set_robot(not is_yellow, 11, 2.0, -2.0, math.radians(0))

    frame_publisher.publish_preset_frame()
    # トピックをsubscribeするためspine_once()を実行
    rclpy.spin_once(observer, timeout_sec=1.0)
    actual_can_pass_id_list, actual_can_shoot_id_list = observer.get_open_path_id_list(my_robot_id=15)
    assert actual_can_pass_id_list == [13]
    assert actual_can_shoot_id_list == []