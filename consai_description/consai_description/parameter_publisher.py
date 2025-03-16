#!/usr/bin/env python3

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

import argparse
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import json
from std_msgs.msg import String
import yaml


class ParameterPublisher(Node):
    def __init__(self, config_rule:str, config_strategy:str, config_control:str):
        super().__init__('parameter_publisher')

        self._logger = self.get_logger()
        self._param_dict = {}
        self._pub_param = {}

        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        def initialize_param(config_file, key):
            self._param_dict[key] = self._declare_parameters_from_yaml(config_file, key)
            self._logger.info(f"Declared parameters: {key} : {self._param_dict[key]}")

            self._pub_param[key] = self.create_publisher(String, 'consai_param/' + key, qos_profile)
            self._publish_parameters(key)

        initialize_param(config_rule, 'rule')
        initialize_param(config_strategy, 'strategy')
        initialize_param(config_control, 'control')

        self.add_on_set_parameters_callback(self._on_parameter_set)

        self._subs_count = {}
        self._timer_check_republish = self.create_timer(1.0, self._check_republish)

    def _declare_parameters_from_yaml(self, yaml_file, prefix='') -> dict:
        # yamlファイルからパラメータを宣言する
        # パースしたyamlファイルのデータ構造を返す
        with open(yaml_file, 'r') as file:
            data = yaml.safe_load(file)

        ros_parameters = data.get('/**', {}).get('ros__parameters', {})
        output_dict = {}
        self._search_and_declare(prefix, ros_parameters, output_dict)

        return output_dict

    def _search_and_declare(self, prefix: str, parameters, output_dict: dict):
        # 再帰的にyamlファイルを探索し、パラメータを宣言する
        for key, value in parameters.items():
            full_key = f"{prefix}.{key}" if prefix else key
            if isinstance(value, dict):
                output_dict[key] = {}
                self._search_and_declare(full_key, value, output_dict[key])
            else:
                output_dict[key] = value
                self.declare_parameter(full_key, value)

    def _publish_parameters(self, key: str):
        # JSON形式でパラメータを配信
        msg = String()
        msg.data = json.dumps(self._param_dict[key])
        self._pub_param[key].publish(msg)
        self._logger.info(f"Published parameters: {key}")

    def _on_parameter_set(self, parameters):
        # パラメータが更新されたら、トピックとして再publishする

        def update_nested_dict(nested_dict, keys, value):
            current = nested_dict
            for key in keys[:-1]:
                current = current[key]
            current[keys[-1]] = value

        for parameter in parameters:
            keys = parameter.name.split('.')
            if not keys[0] in self._param_dict:
                continue
            prefix = keys[0]
            update_nested_dict(self._param_dict[prefix], keys[1:], parameter.value)
            self._publish_parameters(prefix)

        return SetParametersResult(successful=True)

    def _check_republish(self):
        # Subscriberが増減したら、パラメータを再配信する
        for key in self._pub_param.keys():
            subs_count = self._pub_param[key].get_subscription_count()
            if subs_count != self._subs_count.get(key, 0):
                self._publish_parameters(key)
                self._subs_count[key] = subs_count
                self._logger.info(f"New subscriber joins. Republish parameters: {key}")


if __name__ == '__main__':
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument('--config_rule', type=str)
    arg_parser.add_argument('--config_strategy', type=str)
    arg_parser.add_argument('--config_control', type=str)

    args, other_args = arg_parser.parse_known_args()

    rclpy.init(args=other_args)

    node = ParameterPublisher(
        args.config_rule,
        args.config_strategy,
        args.config_control
    )
    rclpy.spin(node)
    rclpy.shutdown()
