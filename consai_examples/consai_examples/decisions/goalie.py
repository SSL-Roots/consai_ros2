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

from decisions.decision_base import DecisionBase

class GoaleDecision(DecisionBase):

    def __init__(self, robot_operator):
        super().__init__(robot_operator)

    def stop(self, robot_id):
        if self._act_id != self.ACT_ID_STOP:
            self._operator.move_to_normalized(robot_id, -1.0, 0.0, 0.0, True)
            self._act_id = self.ACT_ID_STOP

