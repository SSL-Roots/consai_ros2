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

from collections import deque
from consai_visualizer_msgs.msg import Objects as VisObjects
from consai_visualizer_msgs.msg import Color as VisColor
from consai_visualizer_msgs.msg import ShapeAnnotation
from consai_visualizer_msgs.msg import ShapeArc
from consai_visualizer_msgs.msg import ShapeCircle
from consai_visualizer_msgs.msg import ShapeLine
from consai_visualizer_msgs.msg import ShapePoint
from consai_visualizer_msgs.msg import ShapeRectangle
from consai_visualizer_msgs.msg import ShapeRobot
from consai_visualizer_msgs.msg import ShapeTube
import datetime
import math
from python_qt_binding.QtCore import QPointF
from python_qt_binding.QtCore import QRectF
from python_qt_binding.QtCore import QSizeF
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QColor
from python_qt_binding.QtGui import QFontMetrics
from python_qt_binding.QtGui import QPainter
from python_qt_binding.QtGui import QPainterPath
from python_qt_binding.QtGui import QPen
from python_qt_binding.QtWidgets import QWidget
from typing import Dict


class FieldWidget(QWidget):

    def __init__(self, parent=None):
        super(FieldWidget, self).__init__(parent)

        self.setMouseTracking(True)  # マウスカーソルの位置を取得するために必要

        # 定数
        self._MOUSE_WHEEL_ZOOM_RATE = 0.2  # マウスホイール操作による拡大縮小操作量
        self._LIMIT_SCALE = 0.2  # 縮小率の限界値
        self._FULL_FIELD_LENGTH = 12.6
        self._FULL_FIELD_WIDTH = 9.6

        # 外部からセットするパラメータ
        self._logger = None
        self._invert = False
        self._visualizer_objects: Dict[int, Dict[tuple[str, str], VisObjects]] = {}
        self._active_layers: list[tuple[str, str]] = []

        # 内部で変更するパラメータ
        self._draw_area_scale = 1.0  # 描画領域の拡大縮小率
        self._draw_area_offset = QPointF(0.0, 0.0)  # 描画領域のオフセット
        self._draw_area_size = QSizeF(self.rect().size())  # 描画領域サイズ
        self._scale_field_to_draw = 1.0  # フィールド領域から描画領域に縮小するスケール
        self._do_rotate_draw_area = False  # 描画領域を90度回転するフラグ
        self._mouse_clicked_point = QPointF(0.0, 0.0)  # マウスでクリックした描画領域の座標
        self._mouse_current_point = QPointF(0.0, 0.0)  # マウスカーソルの現在座標
        self._mouse_drag_offset = QPointF(0.0, 0.0)  # マウスでドラッグした距離
        self._previous_update_time = datetime.datetime.now()  # 前回の描画時刻
        self._frame_rate_buffer = deque(maxlen=20)  # フレームレート計算用のバッファ
        self._mouse_double_clicked = False  # マウスダブルクリックのフラグ
        self._mouse_double_click_updated = False  # ダブルクリックが更新されたフラグ

    def set_logger(self, logger):
        self._logger = logger

    def set_invert(self, param):
        self._invert = param

    def set_visualizer_objects(self, msg):
        self._visualizer_objects.setdefault(msg.z_order, {})[(msg.layer, msg.sub_layer)] = msg

    def set_active_layers(self, layers: list[tuple[str, str]]):
        self._active_layers = layers

    def get_mouse_double_click_updated(self) -> bool:
        return self._mouse_double_click_updated

    def get_mouse_double_click_points(self) -> tuple[QPointF, QPointF]:
        points = (self._convert_draw_to_field_pos(self._mouse_clicked_point),
                  self._convert_draw_to_field_pos(self._mouse_current_point))
        return points

    def reset_mouse_double_click_updated(self) -> None:
        self._mouse_double_click_updated = False

    def mousePressEvent(self, event):
        # マウスクリック時のイベント
        self._mouse_double_clicked = False

        if event.buttons() == Qt.LeftButton:
            self._mouse_clicked_point = event.localPos()
            self._mouse_current_point = event.localPos()

        elif event.buttons() == Qt.RightButton:
            self._reset_draw_area_offset_and_scale()

        self.update()

    def mouseDoubleClickEvent(self, event):
        # ダブルクリック時のイベント
        # mousePressEvent発行後に実行される

        self._mouse_double_clicked = True
        self._mouse_double_click_updated = False

        if event.buttons() == Qt.LeftButton:
            self._mouse_clicked_point = event.localPos()
            self._mouse_current_point = event.localPos()

        self.update()

    def mouseMoveEvent(self, event):
        # マウス移動時のイベント
        self._mouse_current_point = event.localPos()

        if event.buttons() == Qt.LeftButton:
            if self._mouse_double_clicked:
                # ダブルクリック時はマウスの現在位置を更新する
                self._mouse_current_point = event.localPos()

            else:
                # 描画領域を移動するためのオフセットを計算
                self._mouse_drag_offset = (
                    self._mouse_current_point - self._mouse_clicked_point) / self._draw_area_scale

            self.update()

    def mouseReleaseEvent(self, event):
        # マウスクリック解除時のイベント

        if self._mouse_double_clicked:
            self._mouse_double_clicked = False
            self._mouse_double_click_updated = True

        self._draw_area_offset += self._mouse_drag_offset
        self._mouse_drag_offset = QPointF(0.0, 0.0)  # マウスドラッグ距離の初期化

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

        # 背景色をセット
        painter.setBrush(QColor('darkgreen'))
        painter.drawRect(self.rect())

        painter.save()

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

        draw_caption = ('caption', 'caption') in self._active_layers
        self._draw_objects_on_transformed_area(painter, draw_caption)
        self._draw_visualizer_info_on_transformed_area(painter)

        painter.restore()

        self._draw_objects_on_window_area(painter, draw_caption)
        self._draw_visualizer_info_on_window_area(painter)

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
        field_full_width = self._FULL_FIELD_LENGTH
        field_full_height = self._FULL_FIELD_WIDTH
        field_w_per_h = field_full_width / field_full_height
        field_h_per_w = 1.0 / field_w_per_h

        # 描画領域のサイズを決める
        # Widgetのサイズによって描画領域を回転するか判定する
        if widget_w_per_h >= field_w_per_h:
            # Widgetが横長のとき
            self._draw_area_size = QSizeF(widget_height * field_w_per_h, widget_height)
            self._do_rotate_draw_area = False

        elif widget_w_per_h <= field_h_per_w:
            # Widgetが縦長のとき
            self._draw_area_size = QSizeF(widget_width * field_w_per_h, widget_width)
            self._do_rotate_draw_area = True

        else:
            # 描画回転にヒステリシスをもたせる
            if self._do_rotate_draw_area is True:
                self._draw_area_size = QSizeF(widget_height, widget_height * field_h_per_w)
            else:
                self._draw_area_size = QSizeF(widget_width, widget_width * field_h_per_w)

        self._scale_field_to_draw = self._draw_area_size.width() / field_full_width

    def _draw_text(self, painter: QPainter, pos: QPointF, text: str, font_size: int = 10):
        # 回転を考慮したテキスト描画関数
        painter.save()
        font = painter.font()
        font.setPointSize(font_size)
        painter.setFont(font)
        if self._do_rotate_draw_area:
            tmp = pos.x()
            pos.setX(pos.y())
            pos.setY(-tmp)
            painter.rotate(90)

        painter.drawText(pos, text)
        painter.restore()

    def _to_qcolor(self, color: VisColor):
        if color.name:
            output = QColor(color.name)
        else:
            output = QColor()
            output.setRedF(color.red)
            output.setGreenF(color.green)
            output.setBlueF(color.blue)

        output.setAlphaF(color.alpha)
        return output

    def _draw_objects_on_transformed_area(self, painter: QPainter, draw_caption: bool = False):
        # 描画領域の移動や拡大を考慮した座標系でオブジェクトを描画する
        for z_order in sorted(self._visualizer_objects):
            for active_layer in self._active_layers:
                vis_objects = self._visualizer_objects[z_order].get(active_layer)
                if vis_objects is None:
                    continue

                for shape_point in vis_objects.points:
                    self._draw_shape_point(painter, shape_point, draw_caption)

                for shape_line in vis_objects.lines:
                    self._draw_shape_line(painter, shape_line, draw_caption)

                for shape_arc in vis_objects.arcs:
                    self._draw_shape_arc(painter, shape_arc, draw_caption)

                for shape_rect in vis_objects.rects:
                    self._draw_shape_rect(painter, shape_rect, draw_caption)

                for shape_circle in vis_objects.circles:
                    self._draw_shape_circle(painter, shape_circle, draw_caption)

                for shape_tube in vis_objects.tubes:
                    self._draw_shape_tube(painter, shape_tube, draw_caption)

                for shape_robot in vis_objects.robots:
                    self._draw_shape_robot(painter, shape_robot, draw_caption)

    def _draw_visualizer_info_on_transformed_area(self, painter: QPainter):
        # 描画領域の移動や拡大を考慮した座標系で、ビジュアライザが持つ情報を描画する

        # ドラック時の変位を描画
        if self._mouse_double_clicked:
            drag_line = ShapeLine()
            clicked_point = self._convert_draw_to_field_pos(
                self._mouse_clicked_point, apply_invert=False)
            current_point = self._convert_draw_to_field_pos(
                self._mouse_current_point, apply_invert=False)
            distance = current_point - clicked_point
            theta_deg = math.degrees(math.atan2(distance.y(), distance.x()))
            drag_line.p1.x = clicked_point.x()
            drag_line.p1.y = clicked_point.y()
            drag_line.p2.x = current_point.x()
            drag_line.p2.y = current_point.y()
            drag_line.size = 4
            drag_line.color.name = "lightsalmon"
            drag_line.caption = "dist: {:.1f} : {:.1f}, theta: {:.1f}".format(
                distance.x(), distance.y(), theta_deg)
            self._draw_shape_line(painter, drag_line, True)

    def _draw_objects_on_window_area(self, painter: QPainter, draw_caption: bool = False):
        # ウィンドウ領域にオブジェクトを描画する
        for z_order in sorted(self._visualizer_objects):
            for active_layer in self._active_layers:
                vis_objects = self._visualizer_objects[z_order].get(active_layer)
                if vis_objects is None:
                    continue

                for shape_annotation in vis_objects.annotations:
                    self._draw_shape_annotation(painter, shape_annotation, draw_caption)

    def _draw_visualizer_info_on_window_area(self, painter: QPainter):
        # ビジュアライザが持つ情報を描画する

        # フレームレートを描画
        time_diff = datetime.datetime.now() - self._previous_update_time
        self._frame_rate_buffer.append(1.0 / time_diff.total_seconds())
        average_frame_rate = sum(self._frame_rate_buffer) / self._frame_rate_buffer.maxlen
        self._previous_update_time = datetime.datetime.now()
        annotation = ShapeAnnotation()
        annotation.text = "FPS: {:.1f}".format(average_frame_rate)
        annotation.normalized_x = 0.0
        annotation.normalized_y = 0.95
        annotation.normalized_width = 0.1
        annotation.normalized_height = 0.05
        annotation.color.name = "white"
        self._draw_shape_annotation(painter, annotation)

        # カーソル位置を描画
        cursor_pos = self._convert_draw_to_field_pos(self._mouse_current_point)
        if self._invert:
            annotation.text = "inv"
            annotation.color.name = "lightcoral"
        else:
            annotation.text = "pos"
        annotation.text += " {:.2f} : {:.2f}".format(cursor_pos.x(), cursor_pos.y())
        annotation.normalized_x = 0.1
        self._draw_shape_annotation(painter, annotation)

    def _draw_shape_annotation(
            self, painter: QPainter, shape: ShapeAnnotation, draw_caption: bool = False):
        painter.setPen(QPen(self._to_qcolor(shape.color)))
        # Annotationはフィールドではなくウィンドウ領域の座標系で描画する
        TARGET_WIDTH = shape.normalized_width * self.width()
        TARGET_HEIGHT = shape.normalized_height * self.height()

        if shape.text == "":
            return

        painter.save()

        # TARGET_HEIGHTに合わせてフォントサイズを変更する
        font = painter.font()
        font_metrics = QFontMetrics(font)
        height_fit_point_size = font.pointSizeF() * TARGET_HEIGHT / font_metrics.height()
        font.setPointSizeF(height_fit_point_size)
        font_metrics = QFontMetrics(font)

        # テキストがTARGET_WIDTHをはみ出る場合はフォントサイズを小さくする
        if font_metrics.width(shape.text) > TARGET_WIDTH:
            width_fit_point_size = font.pointSizeF() * \
                TARGET_WIDTH / font_metrics.width(shape.text)
            font.setPointSizeF(width_fit_point_size)
            font_metrics = QFontMetrics(font)

        painter.setFont(font)
        rect = QRectF(
            shape.normalized_x * self.width(),
            shape.normalized_y * self.height(),
            TARGET_WIDTH,
            TARGET_HEIGHT)
        painter.drawText(rect, Qt.AlignCenter, shape.text)

        painter.restore()

    def _draw_shape_point(self, painter: QPainter, shape: ShapePoint, draw_caption: bool = False):
        painter.setPen(QPen(self._to_qcolor(shape.color), shape.size))
        point = self._convert_field_to_draw_point(shape.x, shape.y)
        painter.drawPoint(point)

        if draw_caption:
            self._draw_text(painter, point, shape.caption)

    def _draw_shape_line(self, painter: QPainter, shape: ShapeLine, draw_caption: bool = False):
        painter.setPen(QPen(self._to_qcolor(shape.color), shape.size))
        p1 = self._convert_field_to_draw_point(shape.p1.x, shape.p1.y)
        p2 = self._convert_field_to_draw_point(shape.p2.x, shape.p2.y)
        painter.drawLine(p1, p2)

        # 線の中央にキャプションを描画
        if draw_caption:
            p_mid = self._convert_field_to_draw_point(
                (shape.p1.x + shape.p2.x) * 0.5,
                (shape.p1.y + shape.p2.y) * 0.5)
            self._draw_text(painter, p_mid, shape.caption)

    def _draw_shape_arc(self, painter: QPainter, shape: ShapeArc, draw_caption: bool = False):
        painter.setPen(QPen(self._to_qcolor(shape.color), shape.size))

        top_left = self._convert_field_to_draw_point(
            shape.center.x - shape.radius,
            shape.center.y + shape.radius)
        size = shape.radius * 2 * self._scale_field_to_draw
        rect = QRectF(top_left, QSizeF(size, size))

        # angle must be 1/16 degrees order
        start_angle = int(math.degrees(shape.start_angle) * 16)
        end_angle = int(math.degrees(shape.end_angle) * 16)
        span_angle = end_angle - start_angle
        painter.drawArc(rect, start_angle, span_angle)

        # Arcの中央にキャプションを描画
        if draw_caption:
            center = self._convert_field_to_draw_point(
                shape.center.x, shape.center.y)
            self._draw_text(painter, center, shape.caption)

    def _draw_shape_rect(
            self, painter: QPainter, shape: ShapeRectangle, draw_caption: bool = False):
        painter.setPen(QPen(self._to_qcolor(shape.line_color), shape.line_size))
        painter.setBrush(self._to_qcolor(shape.fill_color))

        half_width = shape.width * 0.5
        half_height = shape.height * 0.5
        top_left = self._convert_field_to_draw_point(
            shape.center.x - half_width,
            shape.center.y + half_height)
        bottom_right = self._convert_field_to_draw_point(
            shape.center.x + half_width,
            shape.center.y - half_height)
        rect = QRectF(top_left, bottom_right)
        painter.drawRect(rect)

        if draw_caption:
            bottom_center = self._convert_field_to_draw_point(
                shape.center.x, shape.center.y - half_height)
            self._draw_text(painter, bottom_center, shape.caption)

    def _draw_shape_circle(
            self, painter: QPainter, shape: ShapeCircle, draw_caption: bool = False):
        painter.setPen(QPen(self._to_qcolor(shape.line_color), shape.line_size))
        painter.setBrush(self._to_qcolor(shape.fill_color))

        center = self._convert_field_to_draw_point(shape.center.x, shape.center.y)
        size = shape.radius * self._scale_field_to_draw
        painter.drawEllipse(center, size, size)

        if draw_caption:
            bottom = self._convert_field_to_draw_point(
                shape.center.x, shape.center.y - shape.radius * 1.2)
            self._draw_text(painter, bottom, shape.caption)

    def _draw_shape_tube(self, painter: QPainter, shape: ShapeTube, draw_caption: bool = False):
        painter.setPen(QPen(self._to_qcolor(shape.line_color), shape.line_size))
        painter.setBrush(self._to_qcolor(shape.fill_color))

        diff_x = shape.p2.x - shape.p1.x
        diff_y = shape.p2.y - shape.p1.y
        theta = math.atan2(diff_y, diff_x)

        top_left = self._convert_field_to_draw_point(
            shape.p1.x + shape.radius * math.cos(theta + math.pi * 0.5),
            shape.p1.y + shape.radius * math.sin(theta + math.pi * 0.5))

        top_right = self._convert_field_to_draw_point(
            shape.p2.x + shape.radius * math.cos(theta + math.pi * 0.5),
            shape.p2.y + shape.radius * math.sin(theta + math.pi * 0.5))

        bottom_right = self._convert_field_to_draw_point(
            shape.p2.x + shape.radius * math.cos(theta - math.pi * 0.5),
            shape.p2.y + shape.radius * math.sin(theta - math.pi * 0.5))

        bottom_left = self._convert_field_to_draw_point(
            shape.p1.x + shape.radius * math.cos(theta - math.pi * 0.5),
            shape.p1.y + shape.radius * math.sin(theta - math.pi * 0.5))

        p1 = self._convert_field_to_draw_point(shape.p1.x, shape.p1.y)
        p2 = self._convert_field_to_draw_point(shape.p2.x, shape.p2.y)
        radius = shape.radius * self._scale_field_to_draw

        path = QPainterPath()
        path.moveTo(top_left)
        path.addEllipse(p1, radius, radius)
        path.moveTo(top_right)
        path.addEllipse(p2, radius, radius)
        path.moveTo(top_left)
        path.lineTo(top_right)
        path.lineTo(bottom_right)
        path.lineTo(bottom_left)
        path.lineTo(top_left)
        path.setFillRule(Qt.WindingFill)
        painter.drawPath(path)

        if draw_caption:
            self._draw_text(painter, bottom_left, shape.caption)

    def _draw_shape_robot(self, painter: QPainter, shape: ShapeRobot, draw_caption: bool = False):
        painter.setPen(QPen(self._to_qcolor(shape.line_color), shape.line_size))
        painter.setBrush(self._to_qcolor(shape.fill_color))

        top_left = self._convert_field_to_draw_point(
            shape.x - shape.radius,
            shape.y + shape.radius)
        size = shape.radius * 2 * self._scale_field_to_draw
        rect = QRectF(top_left, QSizeF(size, size))

        FRONT_ANGLE = 55  # ロボットの前方を描画する角度
        start_angle = int(math.degrees(shape.theta) - FRONT_ANGLE * 0.5) * 16
        span_angle = (-360 + FRONT_ANGLE) * 16
        painter.drawChord(rect, start_angle, span_angle)

        # ロボットID
        FONT_SIZE = int(shape.radius * 111)  # ロボットサイズに比例したフォントサイズ
        text_point = self._convert_field_to_draw_point(
            shape.x - shape.radius * 0.8,
            shape.y - shape.radius * 0.5)
        self._draw_text(painter, text_point, str(shape.id), FONT_SIZE)

        # ロボットの真下にキャプションを描く
        if draw_caption:
            caption_point = self._convert_field_to_draw_point(
                shape.x,
                shape.y - shape.radius * 1.5)
            self._draw_text(painter, caption_point, shape.caption)

    def _convert_field_to_draw_point(self, x, y):
        # フィールド座標系を描画座標系に変換する
        draw_x = x * self._scale_field_to_draw
        draw_y = -y * self._scale_field_to_draw
        point = QPointF(draw_x, draw_y)
        return point

    def _convert_draw_to_field_pos(self, point, apply_invert=True):
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

        # サイド反転の処理
        if self._invert and apply_invert:
            field_x *= -1.0
            field_y *= -1.0

        return QPointF(field_x, field_y)
