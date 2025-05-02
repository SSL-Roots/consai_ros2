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

"""
PlayNodeやAgentSchedulerNodeなどを起動し, 試合の中核処理を実行するモジュール.

ROS2ノードをマルチスレッドで管理しつつ, 一定周期で世界モデルを共有・更新する.
"""

import argparse
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

from consai_game.core.play.play_node import PlayNode
from consai_game.core.tactic.agent_scheduler_node import AgentSchedulerNode
from consai_game.world_model.visualize_msg_publisher_node import VisualizeMsgPublisherNode
from consai_game.world_model.world_model_provider_node import WorldModelProviderNode


def main():
    """世界モデルの情報を各ノードに渡し, 1秒間に指定回数ループ処理を行う関数."""
    while rclpy.ok():
        world_model = world_model_provider_node.world_model
        play_node.set_world_model(world_model)
        agent_scheduler_node.set_world_model(world_model)
        vis_msg_publisher_node.publish(world_model)

        time.sleep(1 / UPDATE_HZ)


if __name__ == "__main__":
    arg_parser = argparse.ArgumentParser()
    PlayNode.add_arguments(arg_parser)

    arg_parser.add_argument("--yellow", type=lambda x: x.lower() == "true", default=False)
    arg_parser.add_argument("--invert", type=lambda x: x.lower() == "true", default=False)

    args, other_args = arg_parser.parse_known_args()
    rclpy.init(args=other_args)

    UPDATE_HZ = 10

    play_node = PlayNode(update_hz=UPDATE_HZ, book_name=args.playbook)
    play_node.select_role_assignment_method(name=args.assign, goalie_id=args.goalie)

    team_is_yellow = args.yellow
    world_model_provider_node = WorldModelProviderNode(
        update_hz=UPDATE_HZ,
        team_is_yellow=team_is_yellow,
        goalie_id=args.goalie,
        invert=args.invert,
    )
    # TODO: agent_numをplay_nodeから取得したい
    agent_scheduler_node = AgentSchedulerNode(update_hz=UPDATE_HZ, team_is_yellow=team_is_yellow, agent_num=11)
    play_node.set_update_role_callback(agent_scheduler_node.set_roles)
    vis_msg_publisher_node = VisualizeMsgPublisherNode()

    logger = rclpy.logging.get_logger("consai_game")

    executor = MultiThreadedExecutor()
    executor.add_node(play_node)
    executor.add_node(world_model_provider_node)
    executor.add_node(agent_scheduler_node)
    executor.add_node(vis_msg_publisher_node)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        main()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor.shutdown()
