#!/usr/bin/env python3
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

from consai_examples.decisions.decision_base import DecisionBase
from consai_examples.operation import Operation
from consai_examples.operation import TargetXY
from consai_examples.operation import TargetTheta


class SubstituteDecision(DecisionBase):

    def __init__(self, robot_operator, field_observer, invert=False):
        super().__init__(robot_operator, field_observer)

        # サイドチェンジに関わらず退避位置を常に同じするためにinvertフラグを取得する
        self._invert = invert

    def _substitute_operation(self):
        # ロボット交代位置
        # ロボットは
        #    フィールドマージンに部分的接触している
        #    かつハーフウェーラインから1m離れない位置
        # で交代できる
        # ただしボールがハーフウェーラインから2m以上離れている場合
        # https://robocup-ssl.github.io/ssl-rules/sslrules.html#_robot_substitution
        target_name = "SUBSTITUTION_POS"
        pos_x = 0.0
        pos_y = -0.3 + -0.08
        if self._invert:
            pos_y *= -1.0
        self._operator.set_named_target(target_name, pos_x, pos_y)
        self._operator.publish_named_targets()
        return Operation().move_to_pose(
            TargetXY.named_target(target_name), TargetTheta.value(0.0))


def gen_substitute_function():
    def function(self, robot_id, placement_pos=None):
        operation = self._substitute_operation()
        self._operator.operate(robot_id, operation)
    return function


for name in ['stop', 'inplay',
             'our_pre_kickoff', 'our_kickoff', 'their_pre_kickoff', 'their_kickoff',
             'our_pre_penalty', 'our_penalty', 'their_pre_penalty', 'their_penalty',
             'our_penalty_inplay', 'their_penalty_inplay',
             'our_direct', 'their_direct', 'our_indirect', 'their_indirect',
             'our_timeout', 'their_timeout']:
    setattr(SubstituteDecision, name, gen_substitute_function())
