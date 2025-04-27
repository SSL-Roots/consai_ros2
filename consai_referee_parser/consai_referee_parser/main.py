#!/usr/bin/env python3
# coding: UTF-8

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

import argparse

import rclpy
from consai_referee_parser.referee_parser_node import RefereeParserNode


if __name__ == "__main__":
    arg_parser = argparse.ArgumentParser()

    arg_parser.add_argument("--yellow", type=lambda x: x.lower() == "true", default=False)
    arg_parser.add_argument("--invert", type=lambda x: x.lower() == "true", default=False)

    args, other_args = arg_parser.parse_known_args()
    rclpy.init(args=other_args)

    parser_node = RefereeParserNode(team_is_yellow=args.yellow, invert=args.invert)

    rclpy.spin(parser_node)

    rclpy.shutdown()
