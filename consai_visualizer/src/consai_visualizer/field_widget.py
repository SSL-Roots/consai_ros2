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

from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtGui import QPainter, QFont, QColor
from python_qt_binding.QtCore import Qt

class FieldWidget(QWidget):

    def __init__(self, parent=None):
        super(FieldWidget, self).__init__(parent)

    def paintEvent(self, event):
        painter = QPainter(self)

        painter.setBrush(Qt.black)
        painter.drawRect(self.rect())

        painter.setPen(Qt.white)
        font = painter.font()
        font.setPointSize(80)
        painter.setFont(font)
        
        # テキストを描画する
        x = 0 # 左端
        y = self.rect().height()*0.5 # 上下の中心
        painter.drawText(x,y, "Hello World!")
