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
from consai_game.core.tactic.agent_scheduler_node import AgentSchedulerNode
from consai_game.core.play.play_node import PlayNode
from consai_game.world_model.world_model_provider_node import WorldModelProviderNode
import rclpy
from rclpy.executors import MultiThreadedExecutor

import threading
import time


def main():
    while rclpy.ok():
        world_model = world_model_provider_node.world_model
        play_node.set_world_model(world_model)
        agent_scheduler_node.set_world_model(world_model)

        time.sleep(1)


if __name__ == '__main__':
    arg_parser = argparse.ArgumentParser()
    PlayNode.add_arguments(arg_parser)

    arg_parser.add_argument('--yellow', type=lambda x: x.lower() == 'true', default=False)

    args, other_args = arg_parser.parse_known_args()
    rclpy.init(args=other_args)

    play_node = PlayNode(update_hz=10, book_name=args.playbook)

    team_is_yellow = args.yellow
    world_model_provider_node = WorldModelProviderNode(
        update_hz=10, team_is_yellow=team_is_yellow)
    # TODO: agent_numをplay_nodeから取得したい
    agent_scheduler_node = AgentSchedulerNode(
        update_hz=10, team_is_yellow=team_is_yellow, agent_num=11)
    play_node.set_update_role_callback(agent_scheduler_node.set_roles)

    logger = rclpy.logging.get_logger('consai_game')

    executor = MultiThreadedExecutor()
    executor.add_node(play_node)
    executor.add_node(world_model_provider_node)
    executor.add_node(agent_scheduler_node)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        main()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor.shutdown()
