# coding: UTF-8

# Copyright 2021 Roots
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

from enum import Enum
import math

from python_qt_binding.QtCore import QPoint
from python_qt_binding.QtCore import QPointF
from python_qt_binding.QtCore import QRect
from python_qt_binding.QtCore import QSize
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QColor
from python_qt_binding.QtGui import QPainter
from python_qt_binding.QtGui import QPen
from python_qt_binding.QtWidgets import QWidget
from robocup_ssl_msgs.msg import BallReplacement
from robocup_ssl_msgs.msg import GeometryFieldSize
from robocup_ssl_msgs.msg import Point as RefPoint
from robocup_ssl_msgs.msg import Replacement
from robocup_ssl_msgs.msg import RobotId
from robocup_ssl_msgs.msg import TrackedFrame


class ClickedObject(Enum):
    IS_NONE = 0
    IS_BALL_POS = 1
    IS_BALL_VEL = 2


class FieldWidget(QWidget):

    def __init__(self, parent=None):
        super(FieldWidget, self).__init__(parent)

        # 定数
        # Ref: named color  https://www.w3.org/TR/SVG11/types.html#ColorKeywords
        self._COLOR_FIELD_CARPET = Qt.green
        self._COLOR_FIELD_LINE = Qt.white
        self._COLOR_BALL = QColor('orange')
        self._COLOR_BLUE_ROBOT = Qt.cyan
        self._COLOR_YELLOW_ROBOT = Qt.yellow
        self._COLOR_REPLACEMENT_POS = QColor('magenta')
        self._COLOR_REPLACEMENT_VEL_ANGLE = QColor('darkviolet')
        self._COLOR_GOAL_POSE = QColor('silver')
        self._COLOR_DESIGNATED_POSITION = QColor('red')
        self._THICKNESS_FIELD_LINE = 2
        self._MOUSE_WHEEL_ZOOM_RATE = 0.2  # マウスホイール操作による拡大縮小操作量
        self._LIMIT_SCALE = 0.2  # 縮小率の限界値
        # Ref: https://robocup-ssl.github.io/ssl-rules/sslrules.html#_ball
        self._RADIUS_BALL = 21.5  # diameter is 43 mm.
        # Ref: https://robocup-ssl.github.io/ssl-rules/sslrules.html#_shape
        self._RADIUS_ROBOT = 90  # diameter is 180 mm.
        self._RADIUS_REPLACEMENT_BALL_POS = self._RADIUS_BALL + 300
        self._RADIUS_REPLACEMENT_BALL_VEL = self._RADIUS_BALL + 500
        self._RADIUS_REPLACEMENT_ROBOT_POS = self._RADIUS_ROBOT + 100
        self._RADIUS_REPLACEMENT_ROBOT_ANGLE = self._RADIUS_ROBOT + 200
        self._GAIN_REPLACE_BALL_VEL = 0.001 * 3.0
        self._MAX_VELOCITY_OF_REPLACE_BALL = 8.0
        self._ID_POS = QPoint(150, 150)  # IDの表示位置 mm.
        self._RADIUS_DESIGNATED_POSITION = 150  # ボールプレースメント成功範囲

        # 外部からセットするパラメータ
        self._logger = None
        self._pub_replacement = None
        self._can_draw_geometry = False
        self._can_draw_detection = False
        self._can_draw_detection_tracked = False
        self._can_draw_replacement = False
        self._field = GeometryFieldSize()
        self._field.field_length = 12000  # resize_draw_area()で0 divisionを防ぐための初期値
        self._field.field_width = 9000  # resize_draw_area()で0 divisionを防ぐための初期値
        self._detections = {}
        self._detection_tracked = TrackedFrame()
        self._goal_poses = {}
        self._designated_position = {}  # ball placementの目標位置

        # 内部で変更するパラメータ
        self._draw_area_scale = 1.0  # 描画領域の拡大縮小率
        self._draw_area_offset = QPointF(0.0, 0.0)  # 描画領域のオフセット
        self._draw_area_size = self.rect().size()  # 描画領域サイズ
        self._scale_field_to_draw = 1.0  # フィールド領域から描画領域に縮小するスケール
        self._do_rotate_draw_area = False  # 描画領域を90度回転するフラグ
        self._mouse_clicked_point = QPointF(0.0, 0.0)  # マウスでクリックした描画領域の座標
        self._mouse_current_point = QPointF(0.0, 0.0)  # マウスカーソルの現在座標
        self._mouse_drag_offset = QPointF(0.0, 0.0)  # マウスでドラッグした距離
        self._clicked_replacement_object = ClickedObject.IS_NONE  # マウスでクリックした、grSimのReplacementの対象

    def set_logger(self, logger):
        self._logger = logger

    def set_pub_replacement(self, pub_replacement):
        self._pub_replacement = pub_replacement

    def set_can_draw_geometry(self, enable=True):
        self._can_draw_geometry = enable

    def set_can_draw_detection(self, enable=True):
        self._can_draw_detection = enable

    def set_can_draw_detection_tracked(self, enable=True):
        self._can_draw_detection_tracked = enable

    def set_can_draw_replacement(self, enable=True):
        self._can_draw_replacement = enable

    def set_field(self, msg):
        self._field = msg.field
        self._resize_draw_area()

    def set_detection(self, msg):
        self._detections[msg.camera_id] = msg

    def set_detection_tracked(self, msg):
        self._detection_tracked = msg

    def set_goal_pose(self, msg, robot_id):
        self._goal_poses[robot_id] = msg

    def set_designated_position(self, msg):
        self._designated_position[0] = msg

    def reset_topics(self):
        # 取得したトピックをリセットする
        self._detections = {}
        self._goal_poses = {}
        self._designated_position = {}

    def mousePressEvent(self, event):
        # マウスクリック時のイベント

        if event.buttons() == Qt.LeftButton:
            self._mouse_clicked_point = event.localPos()
            self._mouse_current_point = event.localPos()

            # ロボット、ボールをクリックしているか
            if self._can_draw_replacement:
                self._clicked_replacement_object = self._get_clicked_replacement_object(
                    self._mouse_clicked_point)

        elif event.buttons() == Qt.RightButton:
            self._reset_draw_area_offset_and_scale()

        self.update()

    def mouseMoveEvent(self, event):
        # マウス移動時のイベント
        self._mouse_current_point = event.localPos()

        if self._clicked_replacement_object != ClickedObject.IS_NONE:
            # Replacement時は何もしない
            pass
        elif event.buttons() == Qt.LeftButton:
            # 描画領域を移動するためのオフセットを計算
            self._mouse_drag_offset = (
                self._mouse_current_point - self._mouse_clicked_point) / self._draw_area_scale

        self.update()

    def mouseReleaseEvent(self, event):
        # マウスクリック解除時のイベント
        # マウスのドラッグ操作で描画領域を移動する

        if self._can_draw_replacement:
            # grSim用のReplacementをpublish
            if self._clicked_replacement_object == ClickedObject.IS_BALL_POS:
                self._publish_ball_pos_replacement()
            elif self._clicked_replacement_object == ClickedObject.IS_BALL_VEL:
                self._publish_ball_vel_replacement()

        self._draw_area_offset += self._mouse_drag_offset
        self._mouse_drag_offset = QPointF(0.0, 0.0)  # マウスドラッグ距離の初期化
        self._clicked_replacement_object = ClickedObject.IS_NONE  # Replacementの初期化

        self.update()

    def wheelEvent(self, event):
        # マウスホイール回転時のイベント
        if event.angleDelta().y() > 0:
            self._draw_area_scale += self._MOUSE_WHEEL_ZOOM_RATE
        else:
            if self._draw_area_scale > self._LIMIT_SCALE:
                self._draw_area_scale -= self._MOUSE_WHEEL_ZOOM_RATE

        self.update()

    def resizeEvent(self, event):
        self._resize_draw_area()

    def paintEvent(self, event):
        painter = QPainter(self)

        # 描画領域の中心をWidgetの中心に持ってくる
        cx = float(self.width()) * 0.5
        cy = float(self.height()) * 0.5
        painter.translate(cx, cy)
        # 描画領域の拡大・縮小
        painter.scale(self._draw_area_scale, self._draw_area_scale)
        # 描画領域の移動
        painter.translate(self._draw_area_offset + self._mouse_drag_offset)
        # 描画領域の回転
        if self._do_rotate_draw_area is True:
            painter.rotate(-90)

        if self._can_draw_geometry:
            self._draw_geometry(painter)

        self._draw_designated_position(painter)

        if self._can_draw_replacement:
            self._draw_replacement(painter)

        if self._can_draw_detection:
            self._draw_detection(painter)

        if self._can_draw_detection_tracked:
            self._draw_detection_tracked(painter)

    def _get_clicked_replacement_object(self, clicked_point):
        # マウスでクリックした位置がボールやロボットに近いか判定する
        # 近ければreplacementと判定する
        field_clicked_pos = self._convert_draw_to_field_pos(clicked_point)

        clicked_object = ClickedObject.IS_NONE
        for detection in self._detections.values():
            for ball in detection.balls:
                clicked_object = self._get_clicked_ball_object(field_clicked_pos, ball)

        return clicked_object

    def _get_clicked_ball_object(self, clicked_pos, ball):
        ball_pos = QPoint(ball.x, ball.y)

        if self._is_clicked(ball_pos, clicked_pos, self._RADIUS_REPLACEMENT_BALL_POS):
            return ClickedObject.IS_BALL_POS
        elif self._is_clicked(ball_pos, clicked_pos, self._RADIUS_REPLACEMENT_BALL_VEL):
            return ClickedObject.IS_BALL_VEL
        else:
            return ClickedObject.IS_NONE

    def _is_clicked(self, pos1, pos2, threshold):
        # フィールド上のオブジェクトをクリックしたか判定する
        diff_pos = pos1 - pos2
        diff_norm = math.hypot(diff_pos.x(), diff_pos.y())

        if diff_norm < threshold:
            return True
        else:
            return False

    def _publish_ball_pos_replacement(self):
        # grSimのBall Replacementの位置をpublsihする
        ball_replacement = BallReplacement()
        pos = self._convert_draw_to_field_pos(self._mouse_current_point)
        ball_replacement.x.append(pos.x() * 0.001)  # mm に変換
        ball_replacement.y.append(pos.y() * 0.001)  # mm に変換
        replacement = Replacement()
        replacement.ball.append(ball_replacement)
        self._pub_replacement.publish(replacement)

    def _publish_ball_vel_replacement(self):
        # grSimのBall Replacementの速度をpublsihする
        ball_replacement = BallReplacement()

        start_pos = self._convert_draw_to_field_pos(self._mouse_clicked_point)
        end_pos = self._convert_draw_to_field_pos(self._mouse_current_point)

        # ball_replacement.x.append(start_pos.x() * 0.001)  # mm に変換
        # ball_replacement.y.append(start_pos.y() * 0.001)  # mm に変換

        diff_pos = end_pos - start_pos
        diff_norm = math.hypot(diff_pos.x(), diff_pos.y())

        velocity_norm = diff_norm * self._GAIN_REPLACE_BALL_VEL
        if velocity_norm > self._MAX_VELOCITY_OF_REPLACE_BALL:
            velocity_norm = self._MAX_VELOCITY_OF_REPLACE_BALL

        angle = math.atan2(diff_pos.y(), diff_pos.x())
        ball_replacement.vx.append(velocity_norm * math.cos(angle))
        ball_replacement.vy.append(velocity_norm * math.sin(angle))

        replacement = Replacement()
        replacement.ball.append(ball_replacement)
        self._pub_replacement.publish(replacement)

    def _reset_draw_area_offset_and_scale(self):
        # 描画領域の移動と拡大・縮小を初期化する
        self._draw_area_offset = QPointF(0.0, 0.0)
        self._draw_area_scale = 1.0
        self._mouse_drag_offset = QPointF(0.0, 0.0)

    def _resize_draw_area(self):
        # Widgetのサイズに合わせて、描画用のフィールドサイズを変更する

        # Widgetの縦横比を算出
        widget_width = float(self.width())
        widget_height = float(self.height())
        widget_w_per_h = widget_width / widget_height

        # フィールドの縦横比を算出
        field_full_width = self._field.field_length + self._field.boundary_width * 2
        field_full_height = self._field.field_width + self._field.boundary_width * 2
        field_w_per_h = field_full_width / field_full_height
        field_h_per_w = 1.0 / field_w_per_h

        # 描画領域のサイズを決める
        # Widgetのサイズによって描画領域を回転するか判定する
        if widget_w_per_h >= field_w_per_h:
            # Widgetが横長のとき
            self._draw_area_size = QSize(widget_height * field_w_per_h, widget_height)
            self._do_rotate_draw_area = False

        elif widget_w_per_h <= field_h_per_w:
            # Widgetが縦長のとき
            self._draw_area_size = QSize(widget_width * field_w_per_h, widget_width)
            self._do_rotate_draw_area = True

        else:
            # 描画回転にヒステリシスをもたせる
            if self._do_rotate_draw_area is True:
                self._draw_area_size = QSize(widget_height, widget_height * field_h_per_w)
            else:
                self._draw_area_size = QSize(widget_width, widget_width * field_h_per_w)

        self._scale_field_to_draw = self._draw_area_size.width() / field_full_width

    def _draw_geometry(self, painter):
        # フィールド形状の描画

        # グリーンカーペットを描画
        painter.setBrush(self._COLOR_FIELD_CARPET)
        rect = QRect(
            QPoint(-self._draw_area_size.width() * 0.5, -self._draw_area_size.height() * 0.5),
            self._draw_area_size
        )
        painter.drawRect(rect)

        # 白線を描画
        painter.setPen(QPen(self._COLOR_FIELD_LINE, self._THICKNESS_FIELD_LINE))
        for line in self._field.field_lines:
            p1 = self._convert_field_to_draw_point(line.p1.x, line.p1.y)
            p2 = self._convert_field_to_draw_point(line.p2.x, line.p2.y)
            painter.drawLine(p1, p2)

        for arc in self._field.field_arcs:
            top_left = self._convert_field_to_draw_point(
                arc.center.x - arc.radius,
                arc.center.y + arc.radius)
            size = arc.radius * 2 * self._scale_field_to_draw

            # angle must be 1/16 degrees order
            start_angle = math.degrees(arc.a1) * 16
            end_angle = math.degrees(arc.a2) * 16
            span_angle = end_angle - start_angle
            painter.drawArc(top_left.x(), top_left.y(), size, size, start_angle, span_angle)

        # ゴールを描画
        painter.setPen(QPen(Qt.black, self._THICKNESS_FIELD_LINE))
        rect = QRect(
            self._convert_field_to_draw_point(
                self._field.field_length * 0.5, self._field.goal_width*0.5),
            QSize(self._field.goal_depth * self._scale_field_to_draw,
                  self._field.goal_width * self._scale_field_to_draw))
        painter.drawRect(rect)

        painter.setPen(QPen(Qt.black, self._THICKNESS_FIELD_LINE))
        rect = QRect(
            self._convert_field_to_draw_point(
                -self._field.field_length * 0.5 - self._field.goal_depth,
                self._field.goal_width*0.5),
            QSize(self._field.goal_depth * self._scale_field_to_draw,
                  self._field.goal_width * self._scale_field_to_draw))
        painter.drawRect(rect)

    def _draw_detection(self, painter):
        # detectionトピックのロボット・ボールの描画

        for detection in self._detections.values():
            for ball in detection.balls:
                self._draw_detection_ball(painter, ball, detection.camera_id)

            for robot in detection.robots_yellow:
                self._draw_detection_yellow_robot(painter, robot, detection.camera_id)

            for robot in detection.robots_blue:
                self._draw_detection_blue_robot(painter, robot, detection.camera_id)

    def _draw_detection_tracked(self, painter):
        # detection_trackedトピックのロボット・ボールの描画

        for ball in self._detection_tracked.balls:
            self._draw_tracked_ball(painter, ball)

        for robot in self._detection_tracked.robots:
            self._draw_tracked_robot(painter, robot)

    def _draw_replacement(self, painter):
        # grSim Replacementの描画
        for detection in self._detections.values():
            for ball in detection.balls:
                self._draw_replacement_ball(painter, ball)

    def _draw_detection_ball(self, painter, ball, camera_id=-1):
        # detectionトピックのボールを描画する
        point = self._convert_field_to_draw_point(ball.x, ball.y)
        size = self._RADIUS_BALL * self._scale_field_to_draw

        painter.setPen(self._COLOR_BALL)
        painter.setBrush(self._COLOR_BALL)
        painter.drawEllipse(point, size, size)

    def _draw_tracked_ball(self, painter, ball):
        VELOCITY_THRESH = 0.3  # m/s
        PAINT_LENGTH = 10.0  # meters

        # detection_trackedトピックのボールを描画する

        # visibilityが下がるほど、色を透明にする
        color_pen = QColor(Qt.black)
        color_brush = QColor(self._COLOR_BALL)
        if len(ball.visibility) > 0:
            color_pen.setAlpha(255 * ball.visibility[0])
            color_brush.setAlpha(255 * ball.visibility[0])
        else:
            color_pen.setAlpha(0)
            color_brush.setAlpha(0)

        painter.setPen(color_pen)
        painter.setBrush(color_brush)
        point_pos = self._convert_field_to_draw_point(
            ball.pos.x * 1000, ball.pos.y * 1000)  # meters to mm
        size = self._RADIUS_BALL * self._scale_field_to_draw
        painter.drawEllipse(point_pos, size, size)

        # ボール速度を描画する
        if len(ball.vel) == 0:
            return
        velocity = ball.vel[0]

        # 速度が小さければ描画しない
        if math.hypot(velocity.x, velocity.y) < VELOCITY_THRESH:
            return
        direction = math.atan2(velocity.y, velocity.x)
        vel_end_x = PAINT_LENGTH * math.cos(direction) + ball.pos.x
        vel_end_y = PAINT_LENGTH * math.sin(direction) + ball.pos.y
        point_vel = self._convert_field_to_draw_point(
            vel_end_x * 1000, vel_end_y * 1000)  # meters to mm

        painter.setPen(QPen(QColor(102, 0, 255), 2))
        painter.drawLine(point_pos, point_vel)

    def _draw_detection_yellow_robot(self, painter, robot, camera_id=-1):
        # detectionトピックの黄色ロボットを描画する
        self._draw_detection_robot(painter, robot, self._COLOR_YELLOW_ROBOT, camera_id)

    def _draw_detection_blue_robot(self, painter, robot, camera_id=-1):
        # detectionトピックの青色ロボットを描画する
        self._draw_detection_robot(painter, robot, self._COLOR_BLUE_ROBOT, camera_id)

    def _draw_detection_robot(self, painter, robot, color, camera_id=-1):
        # ロボットを描画する
        point = self._convert_field_to_draw_point(robot.x, robot.y)
        size = self._RADIUS_ROBOT * self._scale_field_to_draw

        painter.setPen(color)
        painter.setBrush(color)
        painter.drawEllipse(point, size, size)

        painter.setPen(Qt.black)
        # ロボット角度
        if len(robot.orientation) > 0:
            line_x = self._RADIUS_ROBOT * math.cos(robot.orientation[0])
            line_y = self._RADIUS_ROBOT * math.sin(robot.orientation[0])
            line_point = point + self._convert_field_to_draw_point(line_x, line_y)
            painter.drawLine(point, line_point)

        # ロボットID
        if len(robot.robot_id) > 0:
            text_point = point + \
                self._convert_field_to_draw_point(self._ID_POS.x(), self._ID_POS.y())
            painter.drawText(text_point, str(robot.robot_id[0]))

    def _draw_tracked_robot(self, painter, robot):
        # detection_trackedトピックのロボットを描画する
        color_pen = QColor(Qt.black)
        color_brush = QColor(self._COLOR_BLUE_ROBOT)
        if robot.robot_id.team_color == RobotId.TEAM_COLOR_YELLOW:
            color_brush = QColor(self._COLOR_YELLOW_ROBOT)

        # visibilityが下がるほど、色を透明にする
        if len(robot.visibility) > 0:
            color_brush.setAlpha(255 * robot.visibility[0])
            # ペンの色はvisibilityが0になるまで透明度を下げない
            if(robot.visibility[0] > 0.01):
                color_pen.setAlpha(255)
            else:
                color_pen.setAlpha(0)
        else:
            color_pen.setAlpha(0)
            color_brush.setAlpha(0)

        self._draw_goal_pose(painter, robot)

        painter.setPen(color_pen)
        painter.setBrush(color_brush)
        point = self._convert_field_to_draw_point(
            robot.pos.x * 1000, robot.pos.y * 1000)  # meters to mm
        size = self._RADIUS_ROBOT * self._scale_field_to_draw
        painter.drawEllipse(point, size, size)

        # ロボット角度
        line_x = self._RADIUS_ROBOT * math.cos(robot.orientation)
        line_y = self._RADIUS_ROBOT * math.sin(robot.orientation)
        line_point = point + self._convert_field_to_draw_point(line_x, line_y)
        painter.drawLine(point, line_point)

        # ロボットID
        text_point = point + self._convert_field_to_draw_point(self._ID_POS.x(), self._ID_POS.y())
        painter.drawText(text_point, str(robot.robot_id.id))

    def _draw_goal_pose(self, painter, robot):
        # goal_poseトピックを描画する
        # ロボットの情報も参照するため、draw_tracked_robot()から呼び出すこと

        robot_id = robot.robot_id.id

        # goal_poseが存在しなければ終了
        goal_pose = self._goal_poses.get(robot_id)
        if goal_pose is None:
            return

        # team_colorが一致しなければ終了
        robot_is_yellow = robot.robot_id.team_color == RobotId.TEAM_COLOR_YELLOW
        if robot_is_yellow is not goal_pose.team_is_yellow:
            return

        painter.setPen(Qt.black)
        painter.setBrush(self._COLOR_GOAL_POSE)
        # x,y座標
        point = self._convert_field_to_draw_point(
            goal_pose.pose.x * 1000, goal_pose.pose.y * 1000)  # meters to mm
        size = self._RADIUS_ROBOT * self._scale_field_to_draw
        painter.drawEllipse(point, size, size)

        # 角度
        line_x = self._RADIUS_ROBOT * math.cos(goal_pose.pose.theta)
        line_y = self._RADIUS_ROBOT * math.sin(goal_pose.pose.theta)
        line_point = point + self._convert_field_to_draw_point(line_x, line_y)
        painter.drawLine(point, line_point)

        # ロボットID
        text_point = point + self._convert_field_to_draw_point(self._ID_POS.x(), self._ID_POS.y())
        painter.drawText(text_point, str(robot_id))

        # goal_poseとロボットを結ぶ直線を引く
        painter.setPen(QPen(Qt.red, 2))
        robot_point = self._convert_field_to_draw_point(
            robot.pos.x * 1000, robot.pos.y * 1000)  # meters to mm
        painter.drawLine(point, robot_point)

    def _draw_replacement_ball(self, painter, ball):
        # ボールのreplacementを描画する

        # ボール速度replacement判定エリアの描画
        point = self._convert_field_to_draw_point(ball.x, ball.y)
        size = self._RADIUS_REPLACEMENT_BALL_VEL * self._scale_field_to_draw
        painter.setPen(self._COLOR_REPLACEMENT_VEL_ANGLE)
        painter.setBrush(self._COLOR_REPLACEMENT_VEL_ANGLE)
        painter.drawEllipse(point, size, size)

        # ボール位置replacement判定エリアの描画
        point = self._convert_field_to_draw_point(ball.x, ball.y)
        size = self._RADIUS_REPLACEMENT_BALL_POS * self._scale_field_to_draw
        painter.setPen(self._COLOR_REPLACEMENT_POS)
        painter.setBrush(self._COLOR_REPLACEMENT_POS)
        painter.drawEllipse(point, size, size)

        # ボールのreplacement位置の描画
        if self._clicked_replacement_object == ClickedObject.IS_BALL_POS:
            end_point = self._apply_transpose_to_draw_point(self._mouse_current_point)
            painter.setPen(self._COLOR_REPLACEMENT_POS)
            painter.drawLine(point, end_point)

        # ボールのreplacement速度の描画
        if self._clicked_replacement_object == ClickedObject.IS_BALL_VEL:
            end_point = self._apply_transpose_to_draw_point(self._mouse_current_point)
            painter.setPen(self._COLOR_REPLACEMENT_VEL_ANGLE)
            painter.drawLine(point, end_point)

    def _draw_designated_position(self, painter):
        if self._designated_position:
            point = self._convert_field_to_draw_point(
                self._designated_position[0].x, self._designated_position[0].y)
            size = self._RADIUS_DESIGNATED_POSITION * self._scale_field_to_draw

            painter.setPen(self._COLOR_DESIGNATED_POSITION)
            painter.setBrush(self._COLOR_DESIGNATED_POSITION)
            painter.drawEllipse(point, size, size)

    def _convert_field_to_draw_point(self, x, y):
        # フィールド座標系を描画座標系に変換する
        draw_x = x * self._scale_field_to_draw
        draw_y = -y * self._scale_field_to_draw
        point = QPoint(draw_x, draw_y)
        return point

    def _convert_draw_to_field_pos(self, point):
        # 描画座標系をフィールド座標系に変換する

        # スケールを戻す
        field_x = point.x() / self._draw_area_scale
        field_y = point.y() / self._draw_area_scale

        # 移動を戻す
        field_x -= (self._draw_area_offset.x() + self._mouse_drag_offset.x())
        field_y -= (self._draw_area_offset.y() + self._mouse_drag_offset.y())

        # センタリング
        field_x -= self.width() * 0.5 / self._draw_area_scale
        field_y -= self.height() * 0.5 / self._draw_area_scale

        # 描画エリアの回転を反映
        if self._do_rotate_draw_area:
            field_x, field_y = -field_y, field_x

        field_x /= self._scale_field_to_draw
        field_y /= -self._scale_field_to_draw

        return QPoint(field_x, field_y)

    def _apply_transpose_to_draw_point(self, point):
        # Widget上のポイント（左上が0, 0となる座標系）を、
        # translate、scale、rotate適用後のポイントに変換する

        draw_point = QPoint()

        # 描画中心座標変更の適用
        draw_point.setX(point.x() - self.width() * 0.5)
        draw_point.setY(point.y() - self.height() * 0.5)

        # 描画領域の拡大・縮小の適用
        draw_point /= self._draw_area_scale

        # 描画領域の移動の適用
        draw_point -= self._draw_area_offset

        # 描画領域の回転の適用
        if self._do_rotate_draw_area is True:
            draw_point = QPoint(-draw_point.y(), draw_point.x())

        return draw_point
