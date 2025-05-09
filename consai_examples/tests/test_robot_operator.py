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

"""NamedTargetsの設定, 削除, 公開を行うテストモジュール."""

from consai_examples.robot_operator import RobotOperator

from consai_msgs.msg import NamedTargets

import pytest

import rclpy
from rclpy.node import Node


class NamedTargetsSubscriber(Node):
    """NamedTargetsトピックをsubscribeするためのクラス."""

    def __init__(self):
        """NamedTargetsSubscriberの初期化処理を行うコンストラクタ."""
        super().__init__("subscriber")

        self._sub_named_targets = self.create_subscription(
            NamedTargets, "named_targets", self._named_targets_callback, 1
        )
        self.named_targets = NamedTargets()

    def _named_targets_callback(self, msg):
        """NamedTargetsメッセージを受け取ったときに呼び出されるコールバック関数."""
        self.named_targets = msg


@pytest.fixture
def rclpy_init_shutdown():
    """rclpy.init()とrclpy.shutdown()を行うフィクスチャ."""
    print("rclpy.init()")
    rclpy.init()
    yield
    print("rclpy.shutdown()")
    rclpy.shutdown()


def test_セットしたnamed_targetsがpublishされること(rclpy_init_shutdown):
    """named_targetが設定され、正しくpublishされることを確認するテスト."""
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
    """複数のnamed_targetを設定できることを確認するテスト."""
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
    """同じ名前のnamed_targetを設定した場合、データが上書きされることを確認するテスト."""
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
    """セットしたnamed_targetを個別に削除できることを確認するテスト."""
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
    """セットした全てのnamed_targetを一括で削除できることを確認するテスト."""
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
