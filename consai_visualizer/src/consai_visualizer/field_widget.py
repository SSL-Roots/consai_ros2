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
from python_qt_binding.QtCore import Qt, QSize, QRect, QPoint
from robocup_ssl_msgs.msg import GeometryFieldSize

class FieldWidget(QWidget):

    def __init__(self, parent=None):
        super(FieldWidget, self).__init__(parent)

        self._COLOR_FIELD_CARPET = Qt.green
        self._COLOR_FIELD_LINE = Qt.white
        self._THICKNESS_FIELD_LINE = 2

        self._can_draw_geometry = False
        self._field = GeometryFieldSize()
        self._field.field_length = 12000  # resize_draw_area()で0 divisionを防ぐための初期値
        self._field.field_width = 9000  # resize_draw_area()で0 divisionを防ぐための初期値

        self._draw_area_size = self.rect().size()  # 描画領域サイズ
        self._scale_field_to_draw = 1.0  # フィールド領域から描画領域に縮小するスケール
        self._do_rotate_draw_area = False  # 描画領域を90度回転するフラグ

    def set_can_draw_geometry(self, enable=True):
        self._can_draw_geometry = enable

    def set_field(self, field):
        self._field = field
        self._resize_draw_area()

    def resizeEvent(self, event):
        self._resize_draw_area()

    def paintEvent(self, event):
        painter = QPainter(self)

        # 描画領域の中心をWidgetの中心に持ってくる
        cx = float(self.width()) * 0.5
        cy = float(self.height()) * 0.5
        painter.translate(cx,cy)

        if self._do_rotate_draw_area is True:
            painter.rotate(-90)

        if self._can_draw_geometry:
            self._draw_geometry(painter)

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

    def _convert_field_to_draw_point(self, x, y):
        # フィールド座標系を描画座標系に変換する
        draw_x = x * self._scale_field_to_draw
        draw_y = -y * self._scale_field_to_draw
        point = QPoint(draw_x, draw_y)
        return point
