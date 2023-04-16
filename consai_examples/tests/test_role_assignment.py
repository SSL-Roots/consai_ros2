
from consai_examples.role_assignment import RoleAssignment
import pytest
import rclpy
from rclpy.node import Node
from robocup_ssl_msgs.msg import RobotId
from robocup_ssl_msgs.msg import TrackedFrame
from robocup_ssl_msgs.msg import TrackedRobot

class TrackedFramePublisher(Node):
    def __init__(self):
        super().__init__('publisher')

        self._publisher = self.create_publisher(TrackedFrame, 'detection_tracked', 1)

    def _publish(self, frame):
        self._publisher.publish(frame)
        print("Test topic published")

    def publish_empty_data(self):
        frame = TrackedFrame()
        self._publish(frame)

    def publish_valid_robots(self, blue_ids=[], yellow_ids=[]):
        frame = TrackedFrame()
        for blue_id in blue_ids:
            robot = TrackedRobot()
            robot.robot_id.id = blue_id
            robot.robot_id.team_color = RobotId.TEAM_COLOR_BLUE
            robot.visibility.append(1.0)  # ここが0だとフィールドにいない判定になる
            frame.robots.append(robot)

        for yellow_id in yellow_ids:
            robot = TrackedRobot()
            robot.robot_id.id = yellow_id
            robot.robot_id.team_color = RobotId.TEAM_COLOR_YELLOW
            robot.visibility.append(1.0)  # ここが0だとフィールドにいない判定になる
            frame.robots.append(robot)

        self._publish(frame)


@pytest.fixture
def rclpy_init_shutdown():
    print("rclpy.init()")
    rclpy.init()
    yield
    print("rclpy.shutdown()")
    rclpy.shutdown()

@pytest.mark.parametrize("goalie_id", [0, 1, 10, 11])
def test_引数のgoalie_idが正しく設定されること(rclpy_init_shutdown, goalie_id, blue_ids=[0, 1, 10, 11]):
    assignor = RoleAssignment(goalie_id)  # 先頭でインスタンスを作成しないとエラーがでる
    frame_publisher = TrackedFramePublisher()
    frame_publisher.publish_valid_robots(blue_ids)

    # トピックをsubscribeするためspine_once()を実行
    rclpy.spin_once(assignor, timeout_sec=1.0)
    assignor.update_role()
    ROLE_GOALIE = 0
    assert assignor.get_robot_id(ROLE_GOALIE) == goalie_id

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
    ROLE_GOALIE = 0
    assert assignor.get_robot_id(ROLE_GOALIE) == goalie_id

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
