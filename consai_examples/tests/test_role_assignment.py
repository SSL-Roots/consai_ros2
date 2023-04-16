
from consai_examples.role_assignment import RoleAssignment
import pytest
import rclpy
from rclpy.node import Node
from robocup_ssl_msgs.msg import RobotId
from robocup_ssl_msgs.msg import TrackedBall
from robocup_ssl_msgs.msg import TrackedFrame
from robocup_ssl_msgs.msg import TrackedRobot

# TrackedFrameトピックをpublishするためのクラス
class TrackedFramePublisher(Node):
    def __init__(self):
        super().__init__('publisher')

        self._publisher = self.create_publisher(TrackedFrame, 'detection_tracked', 1)
        self._preset_frame = TrackedFrame()

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

    def set_robot_pos(self, is_yellow, robot_id, pos_x, pos_y):
        robot = TrackedRobot()
        robot.robot_id.id = robot_id
        robot.robot_id.team_color = RobotId.TEAM_COLOR_YELLOW if is_yellow else RobotId.TEAM_COLOR_BLUE
        robot.pos.x = pos_x
        robot.pos.y = pos_y
        robot.visibility.append(1.0)
        self._preset_frame.robots.append(robot)

    def set_ball_pos(self, pos_x, pos_y):
        ball = TrackedBall()
        ball.pos.x = pos_x
        ball.pos.y = pos_y
        ball.visibility.append(1.0)
        self._preset_frame.balls.append(ball)

    def publish_preset_frame(self):
        self._publish(self._preset_frame)


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
    assert changed_roles == [2, 3]
    
    active_roles = assignor.get_active_roles()
    assert active_roles == [0, 1, 2, 3]

def test_ボールに一番近いロボットがAttackerになること(rclpy_init_shutdown):
    assignor = RoleAssignment(0)
    frame_publisher = TrackedFramePublisher()
    frame_publisher.set_robot_pos(False, 7, -5.0, 0.0)
    frame_publisher.set_robot_pos(False, 8, -2.0, 0.0)
    frame_publisher.set_robot_pos(False, 9, -1.0, 0.0)
    frame_publisher.set_ball_pos(-2.0, -0.0)
    frame_publisher.publish_preset_frame()

    rclpy.spin_once(assignor, timeout_sec=1.0)
    assignor.update_role()

    ROLE_ATTACKER = 1
    active_roles = assignor.get_active_roles()
    assert active_roles == [ROLE_ATTACKER, 2, 3]
    assert assignor.get_robot_id(ROLE_ATTACKER) == 8
