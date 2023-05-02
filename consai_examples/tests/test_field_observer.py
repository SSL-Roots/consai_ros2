
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


def test_パサーロボットがいないときにget_receiver_robots_idは空リストを返す(rclpy_init_shutdown):
    observer = FieldObserver(our_team_is_yellow=False)
    assert observer.get_receiver_robots_id(my_robot_id=0) == []


def test_パサー以外がいないときにget_receiver_robots_idは空リストを返す(rclpy_init_shutdown):
    observer = FieldObserver(our_team_is_yellow=False)
    frame_publisher = TrackedFramePublisher()
    frame_publisher.publish_valid_robots(blue_ids=[3])

    # トピックをsubscribeするためspine_once()を実行
    rclpy.spin_once(observer, timeout_sec=1.0)

    assert observer.get_receiver_robots_id(my_robot_id=3) == []

def test_パサーより後ろにロボットがいるときget_receiver_robots_idは空リストを返す(rclpy_init_shutdown):

    observer = FieldObserver(our_team_is_yellow=False)
    frame_publisher = TrackedFramePublisher()
    # 右を向いたパサーの左側にロボットを置く
    frame_publisher.set_robot(False, 3, 0.0, 0.0, 0.0)
    frame_publisher.set_robot(False, 1, -2.0, 0.0, math.radians(30))
    frame_publisher.set_robot(False, 4, -4.0, 0.0, math.radians(-120))
    frame_publisher.publish_preset_frame()
    # トピックをsubscribeするためspine_once()を実行
    rclpy.spin_once(observer, timeout_sec=1.0)
    assert observer.get_receiver_robots_id(my_robot_id=3) == []

    # 左を向いたパサーの右側にロボットを置く
    # TODO: 後ろ向きのパス相手検索に対応できたらコメントを解除する
    # frame_publisher.set_robot(False, 3, 0.0, 0.0, math.radians(180))
    # frame_publisher.set_robot(False, 1, 2.0, 0.0, math.radians(30))
    # frame_publisher.set_robot(False, 4, 4.0, 0.0, math.radians(-120))
    # frame_publisher.publish_preset_frame()
    # # トピックをsubscribeするためspine_once()を実行
    # rclpy.spin_once(observer, timeout_sec=1.0)
    # assert observer.get_receiver_robots_id(my_robot_id=3) == []
