
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

