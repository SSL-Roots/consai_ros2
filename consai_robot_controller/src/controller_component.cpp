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

  declare_parameter("team_is_yellow", false);
  declare_parameter("vx_p", 1.5);
  declare_parameter("vx_i", 0.0);
  declare_parameter("vx_d", 0.2);
  declare_parameter("vy_p", 1.5);
  declare_parameter("vy_i", 0.0);
  declare_parameter("vy_d", 0.2);
  declare_parameter("vtheta_p", 1.0);
  declare_parameter("vtheta_i", 0.0);
  declare_parameter("vtheta_d", 0.0);

  team_is_yellow_ = get_parameter("team_is_yellow").get_value<bool>();

  RCLCPP_INFO(this->get_logger(), "is yellow:%d", team_is_yellow_);

  std::string team_color = "blue";
  if(team_is_yellow_){
    team_color = "yellow";
  }

  steady_clock_ = rclcpp::Clock(RCL_STEADY_TIME);

  const int ROBOT_NUM = 16;
  for(int i=0; i<ROBOT_NUM; i++){
    std::string name_space = team_color + std::to_string(i);
    pub_command_.push_back(create_publisher<consai_msgs::msg::RobotCommand>(
      name_space + "/command", 10)
    );

    server_control_.push_back(rclcpp_action::create_server<RobotControl>(
      get_node_base_interface(),
      get_node_clock_interface(),
      get_node_logging_interface(),
      get_node_waitables_interface(),
      name_space + "/control",
      std::bind(&Controller::handle_goal, this, _1, _2, i),
      std::bind(&Controller::handle_cancel, this, _1, i),
      std::bind(&Controller::handle_accepted, this, _1, i))
    );

    last_update_time_.push_back(steady_clock_.now());

    // PID制御器の初期化
    pid_vx_.push_back(std::make_shared<control_toolbox::Pid>(
      get_parameter("vx_p").get_value<double>(),
      get_parameter("vx_i").get_value<double>(),
      get_parameter("vx_d").get_value<double>())
    );
    pid_vy_.push_back(std::make_shared<control_toolbox::Pid>(
      get_parameter("vy_p").get_value<double>(),
      get_parameter("vy_i").get_value<double>(),
      get_parameter("vy_d").get_value<double>())
    );
    pid_vtheta_.push_back(std::make_shared<control_toolbox::Pid>(
      get_parameter("vtheta_p").get_value<double>(),
      get_parameter("vtheta_i").get_value<double>(),
      get_parameter("vtheta_d").get_value<double>())
    );

    // bindでは関数を宣言できなかったので、ラムダ式を使用する
    // Ref: https://github.com/ros2/rclcpp/issues/273#issuecomment-263826519
    timer_pub_control_command_.push_back(
      create_wall_timer(10ms, [this, robot_id = i] () { this->on_timer_pub_control_command(robot_id);}
      )
    );
    timer_pub_control_command_[i]->cancel();  // タイマーを停止
    timer_pub_stop_command_.push_back(
      create_wall_timer(1s, [this, robot_id = i] () { this->on_timer_pub_stop_command(robot_id);}
      )
    );

    last_world_vel_.push_back(State());
    control_enable_.push_back(false);
    need_response_.push_back(false);
  }
  goal_handle_.resize(ROBOT_NUM);

  sub_detection_tracked_ = create_subscription<TrackedFrame>(
    "detection_tracked", 10, std::bind(&Controller::callback_detection_tracked, this, _1));
  sub_geometry_ = create_subscription<GeometryData>(
    "geometry", 10, std::bind(&Controller::callback_geometry, this, _1));
}

void Controller::on_timer_pub_control_command(const unsigned int robot_id)
{
  // 制御器を更新し、コマンドをpublishするタイマーコールバック関数

  // 制御が許可されていない場合は、このタイマーを止めて、停止コマンドタイマーを起動する
  if(control_enable_[robot_id] == false){
    timer_pub_control_command_[robot_id]->cancel();
    timer_pub_stop_command_[robot_id]->reset();
    return;
  }

  auto command_msg = std::make_unique<consai_msgs::msg::RobotCommand>();
  command_msg->robot_id = robot_id;
  command_msg->team_is_yellow = team_is_yellow_;

  // 制御するロボットの情報を得る
  // ロボットの情報が存在しなければ制御を終える
  TrackedRobot my_robot;
  if(!extract_robot(robot_id, team_is_yellow_, my_robot)){
    std::string error_msg = "Failed to extract ID:" + std::to_string(robot_id) + " robot from detection_tracked msg.";
    RCLCPP_WARN(this->get_logger(), error_msg);

    if(need_response_[robot_id]){
      auto result = std::make_shared<RobotControl::Result>();
      result->success = false;
      result->message = error_msg;
      goal_handle_[robot_id]->abort(result);
      need_response_[robot_id] = false;
    }

    control_enable_[robot_id] = false;
    timer_pub_control_command_[robot_id]->cancel();
    timer_pub_stop_command_[robot_id]->reset();
    pub_command_[robot_id]->publish(std::move(command_msg));
    return;
  }

  // 目標値を取得する
  // 目標値を取得できなければ速度0を目標値とする
  State goal_pose;
  State world_vel;
  auto current_time = steady_clock_.now();
  auto duration = current_time - last_update_time_[robot_id];
  if(parse_goal(goal_handle_[robot_id]->get_goal(), goal_pose)){
    // ワールド座標系での目標速度を算出
    world_vel.x = pid_vx_[robot_id]->computeCommand(goal_pose.x - my_robot.pos.x, duration.nanoseconds());
    world_vel.y = pid_vy_[robot_id]->computeCommand(goal_pose.y - my_robot.pos.y, duration.nanoseconds());
    world_vel.theta = pid_vtheta_[robot_id]->computeCommand(normalize_theta(goal_pose.theta - my_robot.orientation), duration.nanoseconds());
  }

  // 最大速度リミットを適用
  world_vel = limit_world_acceleration(world_vel, last_world_vel_[robot_id], duration);
  world_vel = limit_world_velocity(world_vel);

  // ワールド座標系でのxy速度をロボット座標系に変換
  command_msg->velocity_x = std::cos(my_robot.orientation) * world_vel.x + std::sin(my_robot.orientation) * world_vel.y;
  command_msg->velocity_y = -std::sin(my_robot.orientation) * world_vel.x + std::cos(my_robot.orientation) * world_vel.y;
  command_msg->velocity_theta = world_vel.theta;

  // 制御値を出力する
  pub_command_[robot_id]->publish(std::move(command_msg));

  // 制御更新時間と速度を保存する
  last_update_time_[robot_id] = current_time;
  last_world_vel_[robot_id] = world_vel;

  // 途中経過を報告する
  if(need_response_[robot_id]){
    auto feedback = std::make_shared<RobotControl::Feedback>();
    feedback->remaining_x = goal_pose.x - my_robot.pos.x;
    feedback->remaining_y = goal_pose.y - my_robot.pos.y;
    feedback->remaining_theta = normalize_theta(goal_pose.theta - my_robot.orientation);
    if(my_robot.vel.size() > 0 && my_robot.vel_angular.size() > 0){
      feedback->remaining_vel_x = my_robot.vel[0].x;
      feedback->remaining_vel_y = my_robot.vel[0].y;
      feedback->remaining_vel_theta = my_robot.vel_angular[0];
    }
    
    goal_handle_[robot_id]->publish_feedback(feedback);
  }

  // 目標値に到達したら制御を完了する
  if(arrived(my_robot, goal_pose)){
    if(need_response_[robot_id]){
      auto result = std::make_shared<RobotControl::Result>();

      result->success = true;
      result->message = "Success!";
      goal_handle_[robot_id]->succeed(result);
      control_enable_[robot_id] = false;
      need_response_[robot_id] = false;
      timer_pub_control_command_[robot_id]->cancel();
      timer_pub_stop_command_[robot_id]->reset();
    }
  }
}

void Controller::on_timer_pub_stop_command(const unsigned int robot_id)
{
  // 停止コマンドをpublishするタイマーコールバック関数
  // 通信帯域を圧迫しないため、この関数は低周期（例:1s）で実行すること
  auto command_msg = std::make_unique<consai_msgs::msg::RobotCommand>();
  command_msg->robot_id = robot_id;
  command_msg->team_is_yellow = team_is_yellow_;

  pid_vx_[robot_id]->reset();
  pid_vy_[robot_id]->reset();
  pid_vtheta_[robot_id]->reset();
  last_update_time_[robot_id] = steady_clock_.now();
  pub_command_[robot_id]->publish(std::move(command_msg));

  // 制御が許可されたらこのタイマーを止めて、制御タイマーを起動する
  if(control_enable_[robot_id] == true){
    timer_pub_stop_command_[robot_id]->cancel();
    timer_pub_control_command_[robot_id]->reset();
  }
}

void Controller::callback_detection_tracked(const TrackedFrame::SharedPtr msg)
{
  detection_tracked_ = msg;
}

void Controller::callback_geometry(const GeometryData::SharedPtr msg)
{
  geometry_ = msg;
}

rclcpp_action::GoalResponse Controller::handle_goal(const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const RobotControl::Goal> goal, const unsigned int robot_id)
{
  (void)uuid;
  (void)robot_id;

  State goal_pose;
  // 目標値の解析に失敗したらReject
  if(!parse_goal(goal, goal_pose)){
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Controller::handle_cancel(const std::shared_ptr<GoalHandleRobotControl> goal_handle, const unsigned int robot_id)
{
  // キャンセル信号を受け取ったら制御を停止する
  (void)goal_handle;
  control_enable_[robot_id] = false;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void Controller::handle_accepted(std::shared_ptr<GoalHandleRobotControl> goal_handle, const unsigned int robot_id)
{
  // 目標値が承認されたときに実行するハンドラ

  auto goal = goal_handle->get_goal();
  if(goal->keep_control){
    // 目標値に到達しても制御を続ける
    need_response_[robot_id] = false;

    // アクションを完了する
    auto result = std::make_shared<RobotControl::Result>();
    result->success = true;
    result->message = "Success!";
    goal_handle->succeed(result);
  }else{
    need_response_[robot_id] = true;
  }

  control_enable_[robot_id] = true;
  goal_handle_[robot_id] = goal_handle;
}

bool Controller::parse_goal(const std::shared_ptr<const RobotControl::Goal> goal, State & parsed_pose)
{
  // RobotControlのgoalを解析し、目標姿勢を出力する
  // 解析に失敗したらfalseを返す

  if(!parse_constraint(goal->x, parsed_pose, parsed_pose.x)){
    return false;
  }
  if(!parse_constraint(goal->y, parsed_pose, parsed_pose.y)){
    return false;
  }
  // オフセットを加算
  parsed_pose.x += goal->offset_x;
  parsed_pose.y += goal->offset_y;

  if(!parse_constraint(goal->theta, parsed_pose, parsed_pose.theta)){
    return false;
  }

  parsed_pose.theta = normalize_theta(parsed_pose.theta + goal->offset_theta);

  return true;
}

bool Controller::parse_constraint(const ConstraintTarget & target, const State & current_goal_pose, double & parsed_value) const
{
  // ConstraintTargetを解析する
  // ボールやロボットが存在しない等で値が得られなければfalseを返す

  bool retval = false;
  
  // 実数値
  if(target.value.size() > 0){
    parsed_value = target.value[0];
    retval = true;
  }

  State object_pose;
  bool object_is_exist = false;
  // ボールパラメータ
  if(target.target.size() > 0 && target.target_parameter.size() > 0){
    TrackedBall ball;
    if(target.target[0] == ConstraintTarget::TARGET_BALL && extract_ball(ball)){
      object_pose.x = ball.pos.x;
      object_pose.y = ball.pos.y;
      object_is_exist = true;
    }
  }

  // ロボットパラメータ
  if(target.target_id.size() > 0 && target.target.size() > 0 && target.target_parameter.size() > 0){
    TrackedRobot robot;
    if((target.target[0] == ConstraintTarget::TARGET_BLUE_ROBOT && extract_robot(target.target_id[0], false, robot)) || 
       (target.target[0] == ConstraintTarget::TARGET_YELLOW_ROBOT && extract_robot(target.target_id[0], true, robot))){
      object_pose.x = robot.pos.x;
      object_pose.y = robot.pos.y;
      object_pose.theta = robot.orientation;
      object_is_exist = true;
    }
  }

  if(object_is_exist){
    if(target.target_parameter[0] == ConstraintTarget::PARAMETER_X){
      parsed_value = object_pose.x;
      retval = true;
    }else if(target.target_parameter[0] == ConstraintTarget::PARAMETER_Y){
      parsed_value = object_pose.y;
      retval = true;
    }else if(target.target_parameter[0] == ConstraintTarget::PARAMETER_THETA){
      parsed_value = object_pose.theta;
      retval = true;
    }else if(target.target_parameter[0] == ConstraintTarget::PARAMETER_LOOK_TO){
      parsed_value = calc_angle(current_goal_pose, object_pose);
      retval = true;
    }else if(target.target_parameter[0] == ConstraintTarget::PARAMETER_LOOK_FROM){
      parsed_value = calc_angle(object_pose, current_goal_pose);
      retval = true;
    }
  }

  return retval;
}

bool Controller::extract_robot(const unsigned int robot_id, const bool team_is_yellow, TrackedRobot & my_robot) const
{
  // detection_trackedから指定された色とIDのロボット情報を抽出する
  // visibilityが低いときは情報が無いと判定する
  const double VISIBILITY_THRESHOLD = 0.01;
  for(auto robot : detection_tracked_->robots){
    if(robot_id != robot.robot_id.id){
      continue;
    }
    bool is_yellow = team_is_yellow && robot.robot_id.team_color == RobotId::TEAM_COLOR_YELLOW;
    bool is_blue = !team_is_yellow && robot.robot_id.team_color == RobotId::TEAM_COLOR_BLUE;
    if(!is_yellow && !is_blue){
      continue;
    }
    // if((team_is_yellow && robot.robot_id.team_color != RobotId::TEAM_COLOR_YELLOW) &&
    //    (!team_is_yellow && robot.robot_id.team_color != RobotId::TEAM_COLOR_BLUE)){
    //   continue;
    // }
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

bool Controller::extract_ball(TrackedBall & my_ball) const
{
  // detection_trackedからボール情報を抽出する
  // visibilityが低いときは情報が無いと判定する
  const double VISIBILITY_THRESHOLD = 0.01;
  for(auto ball : detection_tracked_->balls){
    if(ball.visibility.size() == 0){
      return false;
    }
    if(ball.visibility[0] < VISIBILITY_THRESHOLD){
      return false;
    }

    my_ball = ball;
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

State Controller::limit_world_acceleration(const State & velocity, const State & last_velocity, const rclcpp::Duration & dt) const
{
  // ワールド座標系のロボット加速度に制限を掛ける
  const double MAX_ACCELERATION_XY = 2.0;  // m/s^2
  const double MAX_ACCELERATION_THETA = 2.0 * M_PI;  // rad/s^2

  State modified_velocity = velocity;
  double acc_x = (velocity.x - last_velocity.x) / dt.seconds();
  double acc_y = (velocity.y - last_velocity.y) / dt.seconds();
  double acc_theta = (velocity.theta - last_velocity.theta) / dt.seconds();

  if(std::fabs(acc_x) > MAX_ACCELERATION_XY){
    modified_velocity.x = last_velocity.x + std::copysign(MAX_ACCELERATION_XY * dt.seconds(), acc_x);
  }
  if(std::fabs(acc_y) > MAX_ACCELERATION_XY){
    modified_velocity.y = last_velocity.y + std::copysign(MAX_ACCELERATION_XY * dt.seconds(), acc_y);
  }
  if(std::fabs(acc_theta) > MAX_ACCELERATION_THETA){
    modified_velocity.theta = last_velocity.theta + std::copysign(MAX_ACCELERATION_THETA * dt.seconds(), acc_theta);
  }

  return modified_velocity;
}

bool Controller::arrived(const TrackedRobot & my_robot, const State & goal_pose)
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
  double diff_x = goal_pose.x - my_robot.pos.x;
  double diff_y = goal_pose.y - my_robot.pos.y;
  double remaining_theta = std::fabs(normalize_theta(goal_pose.theta - my_robot.orientation));
  double remaining_distance = std::hypot(diff_x, diff_y);
  if(remaining_distance < DISTANCE_THRESHOLD && remaining_theta < THETA_THRESHOLD){
    return true;
  }
  return false;
}

double Controller::calc_angle(const State & from_pose, const State & to_pose) const
{
  // from_poseからto_poseへの角度を計算する

  double diff_x = to_pose.x - from_pose.x;
  double diff_y = to_pose.y - from_pose.y;

  return std::atan2(diff_y, diff_x);
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
