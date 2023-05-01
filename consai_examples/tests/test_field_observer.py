
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


def test_ロボットがいないときにget_receiver_robots_idは空リストを返す(rclpy_init_shutdown):
    observer = FieldObserver(our_team_is_yellow=False)
    # frame_publisher = TrackedFramePublisher()
    # # 12台のロボットを用意する
    # frame_publisher.publish_valid_robots(blue_ids=list(range(13)))

    # # トピックをsubscribeするためspine_once()を実行
    # rclpy.spin_once(observer, timeout_sec=1.0)
    assert observer.get_receiver_robots_id(my_robot_id=0) == []

