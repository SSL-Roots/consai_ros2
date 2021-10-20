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


#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

#include "consai_robot_controller/controller_component.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robocup_ssl_msgs/msg/robot_id.hpp"

using namespace std::chrono_literals;

namespace consai_robot_controller
{

Controller::Controller(const rclcpp::NodeOptions & options)
: Node("controller", options)
{
  using namespace std::placeholders;

  declare_parameter("robot_id", 0);
  declare_parameter("team_is_yellow", false);

  robot_id_ = get_parameter("robot_id").get_value<unsigned int>();
  team_is_yellow_ = get_parameter("team_is_yellow").get_value<bool>();

  RCLCPP_INFO(this->get_logger(), "robot_id is %d, is yellow:%d",robot_id_, team_is_yellow_);

  std::string command_name_space = "blue";
  if(team_is_yellow_){
    command_name_space = "yellow";
  }
  command_name_space += std::to_string(robot_id_);
  pub_command_ = create_publisher<consai_msgs::msg::RobotCommand>(command_name_space + "/command", 10);

  server_control_ = rclcpp_action::create_server<RobotControl>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    command_name_space + "/control",
    std::bind(&Controller::handle_goal, this, _1, _2),
    std::bind(&Controller::handle_cancel, this, _1),
    std::bind(&Controller::handle_accepted, this, _1));
  
  // PID制御器の初期化
  pid_vx_ = std::make_shared<control_toolbox::Pid>(1.5, 0.0, 0.2);
  pid_vy_ = std::make_shared<control_toolbox::Pid>(1.5, 0.0, 0.2);
  pid_vtheta_ = std::make_shared<control_toolbox::Pid>(1.0, 0.0, 0.0);

  steady_clock_ = rclcpp::Clock(RCL_STEADY_TIME);
  last_update_time_ = steady_clock_.now();

  sub_detection_tracked_ = create_subscription<TrackedFrame>(
    "detection_tracked", 10, std::bind(&Controller::callback_detection_tracked, this, _1));

  timer_ = create_wall_timer(10ms, std::bind(&Controller::on_timer, this));
}

void Controller::on_timer()
{
  // 制御器を更新し、コマンドを送信する
  auto command_msg = std::make_unique<consai_msgs::msg::RobotCommand>();
  command_msg->robot_id = robot_id_;
  command_msg->team_is_yellow = team_is_yellow_;

  // 制御が許可されていないときは、目標速度0.0を送信する
  if(control_enable_ == false){
    pid_vx_->reset();
    pid_vy_->reset();
    pid_vtheta_->reset();
    last_update_time_ = steady_clock_.now();
    pub_command_->publish(std::move(command_msg));
    return;
  }

  // 制御するロボットの情報を得る
  // ロボットの情報が存在しなければ制御を終える
  TrackedRobot my_robot;
  if(!extract_my_robot(my_robot)){
    std::string error_msg = "Failed to extract my robot from detection_tracked msg.";
    RCLCPP_WARN(this->get_logger(), error_msg);

    control_enable_ = false;

    if(need_response_){
      auto result = std::make_shared<RobotControl::Result>();
      result->success = false;
      result->message = error_msg;
      goal_handle_->abort(result);
      need_response_ = false;
    }
    pub_command_->publish(std::move(command_msg));
    return;
  }

  // 目標値を取得する
  const auto goal = goal_handle_->get_goal();
  auto goal_pose = parse_goal(goal_handle_->get_goal()); 
  double target_x = goal->x.value;
  double target_y = goal->y.value;
  double target_theta = goal->theta.value;

  // ワールド座標系での目標速度を算出
  auto duration = steady_clock_.now().nanoseconds() - last_update_time_.nanoseconds();
  State world_vel;
  world_vel.x = pid_vx_->computeCommand(target_x - my_robot.pos.x, duration);
  world_vel.y = pid_vy_->computeCommand(target_y - my_robot.pos.y, duration);
  world_vel.theta = pid_vtheta_->computeCommand(normalize_theta(target_theta - my_robot.orientation), duration);

  // 最大速度リミットを適用
  world_vel = limit_world_velocity(world_vel);

  // ワールド座標系でのxy速度をロボット座標系に変換
  State local_vel;
  local_vel.x = std::cos(my_robot.orientation) * world_vel.x + std::sin(my_robot.orientation) * world_vel.y;
  local_vel.y = -std::sin(my_robot.orientation) * world_vel.x + std::cos(my_robot.orientation) * world_vel.y;
  local_vel.theta = world_vel.theta;

  command_msg->velocity_x = local_vel.x;
  command_msg->velocity_y = local_vel.y;
  command_msg->velocity_theta = local_vel.theta;

  // 制御値を出力
  last_update_time_ = steady_clock_.now();
  pub_command_->publish(std::move(command_msg));

  // 途中経過を報告する
  if(need_response_){
    auto feedback = std::make_shared<RobotControl::Feedback>();
    feedback->remain_x = target_x - my_robot.pos.x;
    feedback->remain_y = target_y - my_robot.pos.y;
    feedback->remain_theta = target_theta - my_robot.orientation;
    if(my_robot.vel.size() > 0 && my_robot.vel_angular.size() > 0){
      feedback->remain_vel_x = my_robot.vel[0].x;
      feedback->remain_vel_y = my_robot.vel[0].y;
      feedback->remain_vel_theta = my_robot.vel_angular[0];
    }
    
    goal_handle_->publish_feedback(feedback);
  }

  // 目標値に到達したら制御を完了する
  if(arrived(my_robot, target_x, target_y, target_theta)){
    if(need_response_){
      auto result = std::make_shared<RobotControl::Result>();

      result->success = true;
      result->message = "Success!";
      goal_handle_->succeed(result);
      control_enable_ = false;
      need_response_ = false;
    }
  }
}

void Controller::callback_detection_tracked(const TrackedFrame::SharedPtr msg)
{
  detection_tracked_ = msg;
}

rclcpp_action::GoalResponse Controller::handle_goal(const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const RobotControl::Goal> goal)
{
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Controller::handle_cancel(const std::shared_ptr<GoalHandleRobotControl> goal_handle)
{
  // キャンセル信号を受け取ったら制御を停止する
  (void)goal_handle;
  control_enable_ = false;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void Controller::handle_accepted(std::shared_ptr<GoalHandleRobotControl> goal_handle)
{
  control_enable_ = true;
  need_response_ = true;
  goal_handle_ = goal_handle;
}

State Controller::parse_goal(const std::shared_ptr<const RobotControl::Goal> goal) const
{
  // RobotControlのgoalを解析し、目標姿勢を出力する

  State goal_pose;

  // デフォルトではx, y, thetaの値をそのまま目標姿勢に格納する
  goal_pose.x = goal->x.value;
  goal_pose.y = goal->y.value;
  goal_pose.theta = goal->theta.value;

  return goal_pose;
}

bool Controller::extract_my_robot(TrackedRobot & my_robot)
{
  // detection_trackedから自身の情報を抽出する
  // visibilityが低いときは情報が無いと判定する
  const double VISIBILITY_THRESHOLD = 0.01;
  for(auto robot : detection_tracked_->robots){
    if(robot_id_ != robot.robot_id.id){
      continue;
    }
    if((team_is_yellow_ && robot.robot_id.team_color != RobotId::TEAM_COLOR_YELLOW) &&
       (!team_is_yellow_ && robot.robot_id.team_color != RobotId::TEAM_COLOR_BLUE)){
      continue;
    }
    if(robot.visibility.size() == 0){
      return false;
    }
    if(robot.visibility[0] < VISIBILITY_THRESHOLD){
      return false;
    }

    my_robot = robot;
    break;
  }
  return true;
}

State Controller::limit_world_velocity(const State & velocity) const
{
  // ワールド座標系のロボット速度に制限を掛ける
  const double MAX_VELOCITY_XY = 2.0;  // m/s
  const double MAX_VELOCITY_THETA = 2.0 * M_PI;  // rad/s

  State modified_velocity; 
  modified_velocity.x = std::clamp(velocity.x, -MAX_VELOCITY_XY, MAX_VELOCITY_XY);
  modified_velocity.y = std::clamp(velocity.y, -MAX_VELOCITY_XY, MAX_VELOCITY_XY);
  modified_velocity.theta = std::clamp(velocity.theta, -MAX_VELOCITY_THETA, MAX_VELOCITY_THETA);

  return modified_velocity;
}

bool Controller::arrived(const TrackedRobot & my_robot, const double x, const double y, const double theta)
{
  // 目的地に到着したかどうか判定する
  // my_robotが速度データを持っていたら、速度が一定値以下に低下していることも判定する
  const double DISTANCE_THRESHOLD = 0.01;  // meters
  const double THETA_THRESHOLD = 3.0 * M_PI / 180.0;  // radians
  const double VELOCITY_THRESHOLD = 0.1;  // m/s
  const double OMEGA_THRESHOLD = 0.1 * M_PI;  // rad/s


  // 速度が小さくなっているか判定
  if(my_robot.vel.size() > 0 && my_robot.vel_angular.size() > 0){
    if(std::fabs(my_robot.vel[0].x) > VELOCITY_THRESHOLD ||
       std::fabs(my_robot.vel[0].y) > VELOCITY_THRESHOLD ||
       std::fabs(my_robot.vel_angular[0]) > OMEGA_THRESHOLD){
      return false;
    }
  }

  // 直線距離の差分が小さくなっているか判定
  double diff_x = x - my_robot.pos.x;
  double diff_y = y - my_robot.pos.y;
  double distance_theta = std::fabs(normalize_theta(theta - my_robot.orientation));
  double distance = std::hypot(diff_x, diff_y);
  if(distance < DISTANCE_THRESHOLD && distance_theta < THETA_THRESHOLD){
    return true;
  }
  return false;
}

double Controller::normalize_theta(double theta)
{
  // 角度を-pi ~ piに納める
  while(theta >= M_PI) theta -= 2.0 * M_PI;
  while(theta <= -M_PI) theta += 2.0 * M_PI;
  return theta;
}

}  // namespace consai_robot_controller

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(consai_robot_controller::Controller)
