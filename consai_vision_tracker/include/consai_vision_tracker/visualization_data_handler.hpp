// Copyright 2023 Roots
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

#ifndef CONSAI_VISION_TRACKER__VISUALIZATION_DATA_HANDLER_HPP_
#define CONSAI_VISION_TRACKER__VISUALIZATION_DATA_HANDLER_HPP_

#include "consai_visualizer_msgs/msg/objects.hpp"
#include "robocup_ssl_msgs/msg/detection_frame.hpp"
#include "robocup_ssl_msgs/msg/geometry_data.hpp"
#include "robocup_ssl_msgs/msg/tracked_frame.hpp"
#include "rclcpp/rclcpp.hpp"

namespace consai_vision_tracker
{

using VisualizerObjects = consai_visualizer_msgs::msg::Objects;
using DetectionFrame = robocup_ssl_msgs::msg::DetectionFrame;
using TrackedFrame = robocup_ssl_msgs::msg::TrackedFrame;
using GeometryData = robocup_ssl_msgs::msg::GeometryData;

class VisualizationDataHandler
{
 public:
  explicit VisualizationDataHandler(const rclcpp::Publisher<VisualizerObjects>::SharedPtr ptr);
  ~VisualizationDataHandler() = default;

  void publish_vis_detection(const DetectionFrame::SharedPtr msg);
  void publish_vis_geometry(const GeometryData::SharedPtr msg);
  TrackedFrame::UniquePtr publish_vis_tracked(TrackedFrame::UniquePtr msg);

 private:
  rclcpp::Publisher<VisualizerObjects>::SharedPtr pub_vis_objects_;
};

}  // namespace consai_vision_tracker

#endif  // CONSAI_VISION_TRACKER__VISUALIZATION_DATA_HANDLER_HPP_
