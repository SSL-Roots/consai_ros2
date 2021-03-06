// Copyright 2021 Roots
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "consai_observer/observer_component.hpp"

#include <chrono>
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace consai_observer
{

Observer::Observer(const rclcpp::NodeOptions & options)
: Node("observer", options)
{
  timer_ = create_wall_timer(1s, std::bind(&Observer::on_timer, this));
}

void Observer::on_timer()
{
  RCLCPP_INFO(this->get_logger(), "Hello World!");
}

}  // namespace consai_observer

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(consai_observer::Observer)
