
from consai_examples.field_observer import FieldObserver
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
