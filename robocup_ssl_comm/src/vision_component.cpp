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

#include <chrono>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robocup_ssl_comm/vision_component.hpp"
#include "robocup_ssl_msgs/messages_robocup_ssl_wrapper.pb.h"
#include "robocup_ssl_msgs/msg/detection_frame.hpp"
#include "robocup_ssl_msgs/msg/geometry_data.hpp"

using namespace std::chrono_literals;

namespace robocup_ssl_comm
{

Vision::Vision(const rclcpp::NodeOptions & options)
: Node("vision", options)
{
  timer_ = create_wall_timer(10ms, std::bind(&Vision::on_timer, this));

  receiver_ = std::make_unique<multicast::MulticastReceiver>("224.5.23.2", 10006);
}

void Vision::on_timer()
{
  while(receiver_->available()){
    std::vector<char> buf(2048);
    const size_t size = receiver_->receive(buf);

    if(size > 0){
      SSL_WrapperPacket packet;
      packet.ParseFromString(std::string(buf.begin(), buf.end()));

      if(packet.has_detection())
        RCLCPP_INFO(this->get_logger(), "Has Detection!");
        
      if(packet.has_geometry())
        RCLCPP_INFO(this->get_logger(), "Has Geometry!");
    }
  }
}

}  // namespace robocup_ssl_comm

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(robocup_ssl_comm::Vision)
