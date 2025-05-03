#!/usr/bin/env python3
# coding: UTF-8
#
# Copyright 2023 Roots
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

from consai_tools.hysteresis import Hysteresis


def test_hysteresis():

    h = Hysteresis(off_threshold=-0.1, on_threshold=0.1, initial_state=False)
    assert h.update(0.0) is False
    assert h.update(0.1) is False
    assert h.update(0.2) is True
    assert h.update(0.1) is True
    assert h.update(0.0) is True
    assert h.update(-0.1) is True
    assert h.update(-0.2) is False
    assert h.update(-0.1) is False
