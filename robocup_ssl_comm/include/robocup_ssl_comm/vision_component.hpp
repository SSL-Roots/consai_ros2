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

#ifndef ROBOCUP_SSL_COMM__VISION_COMPONENT_HPP_
#define ROBOCUP_SSL_COMM__VISION_COMPONENT_HPP_

#include "robocup_ssl_comm/visibility_control.h"
#include "rclcpp/rclcpp.hpp"

namespace robocup_ssl_comm
{

class Vision : public rclcpp::Node
{
public:
  ROBOCUP_SSL_COMM_PUBLIC
  explicit Vision(const rclcpp::NodeOptions & options);

protected:
  void on_timer();

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace robocup_ssl_comm

#endif  // ROBOCUP_SSL_COMM__VISION_COMPONENT_HPP_
