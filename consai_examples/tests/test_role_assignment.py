
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

    def publish_robot_exists(self, blue_ids=[], yellow_ids=[]):
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
def test_goalieが正しく設定されるか(rclpy_init_shutdown, goalie_id, blue_ids=[0, 1, 10, 11]):
    assignor = RoleAssignment(goalie_id)  # 先頭でインスタンスを作成しないとエラーがでる
    frame_publisher = TrackedFramePublisher()
    frame_publisher.publish_robot_exists(blue_ids)

    # トピックをsubscribeするためspine_once()を実行
    rclpy.spin_once(assignor, timeout_sec=1.0)
    assignor.update_role()
    ROLE_GOALIE = 0
    assert assignor.get_robot_id(ROLE_GOALIE) == goalie_id

@pytest.mark.parametrize("goalie_id, is_yellow", [(1, False), (3, True)])
def test_team_colorが正しく設定されるか(rclpy_init_shutdown, goalie_id, is_yellow):
    assignor = RoleAssignment(
        goalie_id=goalie_id,
        our_team_is_yellow=is_yellow)
    frame_publisher = TrackedFramePublisher()
    frame_publisher.publish_robot_exists(blue_ids=[1], yellow_ids=[3])

    # トピックをsubscribeするためspine_once()を実行
    rclpy.spin_once(assignor, timeout_sec=1.0)
    assignor.update_role()
    ROLE_GOALIE = 0
    assert assignor.get_robot_id(ROLE_GOALIE) == goalie_id
