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

from decisions.zone_defese_base import ZoneDefenseDecisionBase
from decisions.zone_defese_base import ZoneDefenseID

class Zone1Decision(ZoneDefenseDecisionBase):

    def __init__(self, robot_operator, field_observer, zone_id: ZoneDefenseID):
        super().__init__(robot_operator, field_observer, zone_id)
