# Copyright 2025 Roots
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


from consai_msgs.msg import State2D
from dataclasses import dataclass
import math


@dataclass
class Point:
    """2D point class."""

    x: float
    y: float


@dataclass
class Rectangle:
    """2D rectangle class."""

    top_left: Point
    top_right: Point
    bottom_left: Point
    bottom_right: Point


def state2d_norm(state: State2D) -> float:
    """Return the norm of a State2D object."""
    return math.sqrt(state.x ** 2 + state.y ** 2)
