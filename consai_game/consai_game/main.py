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
from consai_game.play_node import PlayNode
from consai_game.world_model.world_model_provider_node import WorldModelProviderNode
from consai_game.utils.process_info import process_info
import rclpy
from rclpy.executors import MultiThreadedExecutor

import threading
import time


def main():
    while rclpy.ok():
        print(f'Main update, {process_info()}')
        play_node.set_world_model(world_model_provider_node.world_model)
        time.sleep(1)


if __name__ == '__main__':

    arg_parser = argparse.ArgumentParser()
    PlayNode.add_arguments(arg_parser)

    args, other_args = arg_parser.parse_known_args()
    rclpy.init(args=other_args)

    play_node = PlayNode(update_hz=10, playbook_dir=args.playbook)
    world_model_provider_node = WorldModelProviderNode(update_hz=10)

    executor = MultiThreadedExecutor()
    executor.add_node(play_node)
    executor.add_node(world_model_provider_node)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        main()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor.shutdown()
