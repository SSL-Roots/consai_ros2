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

import math
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtGui import QPainter, QPen, QFont, QColor
from python_qt_binding.QtCore import Qt, QSize, QRect, QPoint, QPointF
from robocup_ssl_msgs.msg import DetectionFrame
from robocup_ssl_msgs.msg import GeometryFieldSize

class FieldWidget(QWidget):

    def __init__(self, parent=None):
        super(FieldWidget, self).__init__(parent)

        self._COLOR_FIELD_CARPET = Qt.green
        self._COLOR_FIELD_LINE = Qt.white
        self._COLOR_BALL = QColor("orange")
        self._COLOR_BLUE_ROBOT = Qt.cyan
        self._COLOR_YELLOW_ROBOT = Qt.yellow
        self._THICKNESS_FIELD_LINE = 2
        self._MOUSE_WHEEL_ZOOM_RATE = 0.2  # マウスホイール操作による拡大縮小操作量
        self._LIMIT_SCALE = 0.2  # 縮小率の限界値
        self._RADIUS_BALL = 21.5  # diameter is 43 mm. Ref: https://robocup-ssl.github.io/ssl-rules/sslrules.html#_ball
        self._RADIUS_ROBOT = 90  # diameter is 180 mm. Ref: https://robocup-ssl.github.io/ssl-rules/sslrules.html#_shape
        self._ID_POS = QPoint(150, 150)  # IDの表示位置 mm.

        self._can_draw_geometry = False
        self._can_draw_detection = False
        self._field = GeometryFieldSize()
        self._field.field_length = 12000  # resize_draw_area()で0 divisionを防ぐための初期値
        self._field.field_width = 9000  # resize_draw_area()で0 divisionを防ぐための初期値
        self._detections = {}

        self._draw_area_scale = QPointF(1.0, 1.0)  # 描画領域の拡大縮小率
        self._draw_area_offset = QPointF(0.0, 0.0)  # 描画領域のオフセット
        self._draw_area_size = self.rect().size()  # 描画領域サイズ
        self._scale_field_to_draw = 1.0  # フィールド領域から描画領域に縮小するスケール
        self._do_rotate_draw_area = False  # 描画領域を90度回転するフラグ
        self._mouse_clicked_pos = QPointF(0.0, 0.0)  # マウスでクリックした描画領域の座標
        self._mouse_current_pos = QPointF(0.0, 0.0)  # マウスカーソルの現在座標
        self._mouse_drag_offset = QPointF(0.0, 0.0)  # マウスでドラッグした距離

    def set_can_draw_geometry(self, enable=True):
        self._can_draw_geometry = enable

    def set_can_draw_detection(self, enable=True):
        self._can_draw_detection = enable

    def set_field(self, field):
        self._field = field
        self._resize_draw_area()

    def set_detection(self, detection):
        self._detections[detection.camera_id] = detection

    def mousePressEvent(self, event):
        # マウスクリック時のイベント

        if event.buttons() == Qt.LeftButton:
            self._mouse_clicked_pos = event.localPos()

        elif event.buttons() == Qt.RightButton:
            self._reset_draw_area_offset_and_scale()

        self.update()

    def mouseMoveEvent(self, event):
        # マウス移動時のイベント
        self._mouse_current_pos = event.localPos()

        if event.buttons() == Qt.LeftButton:
            self._mouse_drag_offset = (
                self._mouse_current_pos - self._mouse_clicked_pos) / self._draw_area_scale.x()

        self.update()

    def mouseReleaseEvent(self, event):
        # マウスクリック解除時のイベント
        # マウスのドラッグ操作で描画領域を移動する

        self._draw_area_offset += self._mouse_drag_offset
        self._mouse_drag_offset = QPointF(0.0, 0.0)  # マウスドラッグ距離の初期化

        self.update()

    def wheelEvent(self, event):
        # マウスホイール回転時のイベント
        scale_x = self._draw_area_scale.x()
        if event.angleDelta().y() > 0:
            self._draw_area_scale.setX(scale_x + self._MOUSE_WHEEL_ZOOM_RATE)
            self._draw_area_scale.setY(scale_x + self._MOUSE_WHEEL_ZOOM_RATE)
        else:
            if scale_x > self._LIMIT_SCALE:
                self._draw_area_scale.setX(scale_x - self._MOUSE_WHEEL_ZOOM_RATE)
                self._draw_area_scale.setY(scale_x - self._MOUSE_WHEEL_ZOOM_RATE)
        
        self.update()

    def resizeEvent(self, event):
        self._resize_draw_area()

    def paintEvent(self, event):
        painter = QPainter(self)

        # 描画領域の中心をWidgetの中心に持ってくる
        cx = float(self.width()) * 0.5
        cy = float(self.height()) * 0.5
        painter.translate(cx,cy)
        # 描画領域の拡大・縮小
        painter.scale(self._draw_area_scale.x(), self._draw_area_scale.y())
        # 描画領域の移動
        painter.translate(self._draw_area_offset + self._mouse_drag_offset)
        # 描画領域の回転
        if self._do_rotate_draw_area is True:
            painter.rotate(-90)

        if self._can_draw_geometry:
            self._draw_geometry(painter)

        if self._can_draw_detection:
            self._draw_detection(painter)

    def _reset_draw_area_offset_and_scale(self):
        # 描画領域の移動と拡大・縮小を初期化する
        self._draw_area_offset = QPointF(0.0, 0.0)
        self._draw_area_scale = QPointF(1.0, 1.0)
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

    def _draw_detection(self, painter):
        # ロボット・ボールの描画

        for detection in self._detections.values():
            for ball in detection.balls:
                self._draw_ball(painter, ball, detection.camera_id)
            
            for robot in detection.robots_yellow:
                self._draw_yellow_robot(painter, robot, detection.camera_id)

            for robot in detection.robots_blue:
                self._draw_blue_robot(painter, robot, detection.camera_id)

    def _draw_ball(self, painter, ball, camera_id=-1):
        # ボールを描画する
        point = self._convert_field_to_draw_point(ball.x, ball.y)
        size = self._RADIUS_BALL * self._scale_field_to_draw

        painter.setPen(Qt.black)
        painter.setBrush(self._COLOR_BALL)
        painter.drawEllipse(point, size, size)

    def _draw_yellow_robot(self, painter, robot, camera_id=-1):
        # 黄色ロボットを描画する
        self._draw_robot(painter, robot, self._COLOR_YELLOW_ROBOT, camera_id)

    def _draw_blue_robot(self, painter, robot, camera_id=-1):
        # 青色ロボットを描画する
        self._draw_robot(painter, robot, self._COLOR_BLUE_ROBOT, camera_id)

    def _draw_robot(self, painter, robot, color, camera_id=-1):
        # ロボットを描画する
        point = self._convert_field_to_draw_point(robot.x, robot.y)
        size = self._RADIUS_ROBOT * self._scale_field_to_draw

        painter.setPen(Qt.black)
        painter.setBrush(color)
        painter.drawEllipse(point, size, size)

        # ロボット角度
        if len(robot.orientation) > 0:
            line_x = self._RADIUS_ROBOT * math.cos(robot.orientation[0])
            line_y = self._RADIUS_ROBOT * math.sin(robot.orientation[0])
            line_point = point + self._convert_field_to_draw_point(line_x, line_y)
            painter.drawLine(point, line_point)

        # ロボットID
        if len(robot.robot_id) > 0:
            text_point = point + self._convert_field_to_draw_point(self._ID_POS.x(), self._ID_POS.y())
            painter.drawText(text_point, str(robot.robot_id[0]))

    def _convert_field_to_draw_point(self, x, y):
        # フィールド座標系を描画座標系に変換する
        draw_x = x * self._scale_field_to_draw
        draw_y = -y * self._scale_field_to_draw
        point = QPoint(draw_x, draw_y)
        return point
