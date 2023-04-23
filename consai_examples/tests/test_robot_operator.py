from consai_examples.robot_operator import RobotOperator
from consai_msgs.msg import NamedTargets
import pytest
import rclpy
from rclpy.node import Node

# NamedTargetsトピックをsubscribeするためのクラス
class NamedTargetsSubscriber(Node):
    def __init__(self):
        super().__init__('subscriber')

        self._sub_named_targets = self.create_subscription(
            NamedTargets, 'named_targets', self._named_targets_callback, 1)
        self.named_targets = NamedTargets()

    def _named_targets_callback(self, msg):
        self.named_targets = msg


@pytest.fixture
def rclpy_init_shutdown():
    print("rclpy.init()")
    rclpy.init()
    yield
    print("rclpy.shutdown()")
    rclpy.shutdown()


def test_セットしたnamed_targetsがpublishされること(rclpy_init_shutdown):
    operator = RobotOperator()
    subscriber = NamedTargetsSubscriber()

    operator.set_named_target("test", x=1.2, y=3.4, theta=5.6)
    operator.publish_named_targets()

    # トピックをsubscribeするためspine_once()を実行
    rclpy.spin_once(subscriber, timeout_sec=1.0)
    assert len(subscriber.named_targets.name) == 1
    assert subscriber.named_targets.name[0] == "test"
    assert subscriber.named_targets.pose[0].x == 1.2
    assert subscriber.named_targets.pose[0].y == 3.4
    assert subscriber.named_targets.pose[0].theta == 5.6


def test_複数のnamed_targetsをセットできること(rclpy_init_shutdown):
    operator = RobotOperator()
    subscriber = NamedTargetsSubscriber()

    operator.set_named_target("test1", 0.0, 0.0)
    operator.set_named_target("test2", 0.0, 0.0)
    operator.set_named_target("test3", 0.0, 0.0)
    operator.publish_named_targets()

    # トピックをsubscribeするためspine_once()を実行
    rclpy.spin_once(subscriber, timeout_sec=1.0)
    assert len(subscriber.named_targets.name) == 3


def test_同じ名前のnamed_targetsをセットした場合はデータを上書きすること(rclpy_init_shutdown):
    operator = RobotOperator()
    subscriber = NamedTargetsSubscriber()

    operator.set_named_target("test", 0.0, 0.0)
    operator.set_named_target("test", 1.2, 3.4)
    operator.publish_named_targets()

    # トピックをsubscribeするためspine_once()を実行
    rclpy.spin_once(subscriber, timeout_sec=1.0)
    assert len(subscriber.named_targets.name) == 1
    assert subscriber.named_targets.name[0] == "test"
    assert subscriber.named_targets.pose[0].x == 1.2
    assert subscriber.named_targets.pose[0].y == 3.4


def test_セットしたnamed_targetを個別に削除できること(rclpy_init_shutdown):
    operator = RobotOperator()
    subscriber = NamedTargetsSubscriber()

    operator.set_named_target("test1", 0.0, 0.0)
    operator.set_named_target("test2", 0.0, 0.0)
    operator.set_named_target("test3", 0.0, 0.0)

    operator.remove_named_target("test1")
    # 存在しないnameを指定してもエラーが出ないこと
    operator.remove_named_target("hoge")
    operator.publish_named_targets()

    # トピックをsubscribeするためspine_once()を実行
    rclpy.spin_once(subscriber, timeout_sec=1.0)
    assert len(subscriber.named_targets.name) == 2

def test_セットしたnamed_targetsを一括削除できること(rclpy_init_shutdown):
    operator = RobotOperator()
    subscriber = NamedTargetsSubscriber()

    operator.set_named_target("test1", 0.0, 0.0)
    operator.set_named_target("test2", 0.0, 0.0)
    operator.set_named_target("test3", 0.0, 0.0)

    operator.clear_named_targets()
    operator.publish_named_targets()

    # トピックをsubscribeするためspine_once()を実行
    rclpy.spin_once(subscriber, timeout_sec=1.0)
    assert len(subscriber.named_targets.name) == 0
