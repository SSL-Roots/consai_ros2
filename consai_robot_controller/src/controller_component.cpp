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
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "consai_robot_controller/controller_component.hpp"
#include "consai_robot_controller/geometry_tools.hpp"
#include "consai_robot_controller/control_tools.hpp"
#include "rclcpp/rclcpp.hpp"

namespace consai_robot_controller
{

using namespace std::chrono_literals;
namespace tools = geometry_tools;
namespace ctools = control_tools;

const int ROBOT_NUM = 16;
const auto PARAM_THRESHOLD_LOOKING_BALL_DISTANCE =
  "control_ball_threshold_looking_ball_distance_m";
const auto PARAM_THRESHOLD_LOOKING_BALL_THETA =
  "control_ball_threshold_looking_ball_theta_deg";
const auto PARAM_THRESHOLD_CAN_DRIBBLE_DISTANCE =
  "control_ball_threshold_can_dribble_distance_m";
const auto PARAM_THRESHOLD_CAN_SHOOT_THETA = "control_ball_threshold_can_shoot_theta_deg";
const auto PARAM_DISTANCE_TO_LOOK_BALL = "control_ball_distance_to_look_ball_m";
const auto PARAM_DISTANCE_TO_ROTATE = "control_ball_distance_to_rotate_m";

Controller::Controller(const rclcpp::NodeOptions & options)
: Node("controller", options)
{
  using namespace std::placeholders;

  declare_parameter("team_is_yellow", false);
  declare_parameter("invert", false);
  declare_parameter("max_acceleration_xy", 2.0);
  declare_parameter("max_acceleration_theta", 2.0 * M_PI);
  declare_parameter("max_velocity_xy", 2.0);
  declare_parameter("max_velocity_theta", 2.0 * M_PI);
  declare_parameter("control_range_xy", 1.0);
  declare_parameter("control_a_xy", 1.0);
  declare_parameter("control_a_theta", 0.5);
  declare_parameter(PARAM_THRESHOLD_LOOKING_BALL_DISTANCE, 0.4);
  declare_parameter(PARAM_THRESHOLD_LOOKING_BALL_THETA, 30.0);
  declare_parameter(PARAM_THRESHOLD_CAN_DRIBBLE_DISTANCE, 0.7);
  declare_parameter(PARAM_THRESHOLD_CAN_SHOOT_THETA, 5.0);
  declare_parameter(PARAM_DISTANCE_TO_LOOK_BALL, -0.05);
  declare_parameter(PARAM_DISTANCE_TO_ROTATE, 0.3);
  max_acceleration_xy_ = get_parameter("max_acceleration_xy").get_value<double>();
  max_acceleration_theta_ = get_parameter("max_acceleration_theta").get_value<double>();
  max_velocity_xy_ = get_parameter("max_velocity_xy").get_value<double>();
  max_velocity_theta_ = get_parameter("max_velocity_theta").get_value<double>();

  param_control_range_xy_ = get_parameter("control_range_xy").get_value<double>();
  param_control_a_xy_ = get_parameter("control_a_xy").get_value<double>();
  param_control_a_theta_ = get_parameter("control_a_theta").get_value<double>();

  parser_.param_threshold_looking_ball_distance = get_parameter(
    PARAM_THRESHOLD_LOOKING_BALL_DISTANCE).get_value<double>();
  parser_.param_threshold_looking_ball_theta = get_parameter(
    PARAM_THRESHOLD_LOOKING_BALL_THETA).get_value<double>();
  parser_.param_can_dribble_distance = get_parameter(
    PARAM_THRESHOLD_CAN_DRIBBLE_DISTANCE).get_value<double>();
  parser_.param_can_shoot_theta = get_parameter(
    PARAM_THRESHOLD_CAN_SHOOT_THETA).get_value<double>();
  parser_.param_distance_to_look_ball = get_parameter(
    PARAM_DISTANCE_TO_LOOK_BALL).get_value<double>();
  parser_.param_distance_to_rotate = get_parameter(
    PARAM_DISTANCE_TO_ROTATE).get_value<double>();

  parser_.set_invert(get_parameter("invert").get_value<bool>());
  parser_.set_team_is_yellow(get_parameter("team_is_yellow").get_value<bool>());
  team_is_yellow_ = get_parameter("team_is_yellow").get_value<bool>();

  RCLCPP_INFO(this->get_logger(), "is yellow:%d", team_is_yellow_);

  std::string team_color = "blue";
  if (team_is_yellow_) {
    team_color = "yellow";
  }

  steady_clock_ = rclcpp::Clock(RCL_STEADY_TIME);

  for (int i = 0; i < ROBOT_NUM; i++) {
    pub_command_.push_back(
      create_publisher<RobotCommand>(
        "robot" + std::to_string(i) + "/command", 10)
    );

    std::string name_space = team_color + std::to_string(i);
    server_control_.push_back(
      rclcpp_action::create_server<RobotControl>(
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

    // bindでは関数を宣言できなかったので、ラムダ式を使用する
    // Ref: https://github.com/ros2/rclcpp/issues/273#issuecomment-263826519
    timer_pub_control_command_.push_back(
      create_wall_timer(
        10ms, [this, robot_id = i]() {this->on_timer_pub_control_command(robot_id);}
      )
    );
    timer_pub_control_command_[i]->cancel();  // タイマーを停止
    timer_pub_stop_command_.push_back(
      create_wall_timer(
        100ms, [this, robot_id = i]() {this->on_timer_pub_stop_command(robot_id);}
      )
    );

    last_world_vel_.push_back(State());
    control_enable_.push_back(false);
    need_response_.push_back(false);
  }
  goal_handle_.resize(ROBOT_NUM);

  pub_goal_poses_ = create_publisher<GoalPoses>("goal_poses", 10);
  pub_final_goal_poses_ = create_publisher<GoalPoses>("final_goal_poses", 10);
  vis_data_handler_ = std::make_shared<VisualizationDataHandler>(
    create_publisher<VisualizerObjects>(
    "visualizer_objects", rclcpp::SensorDataQoS()));
  timer_pub_goal_poses_ =
    create_wall_timer(10ms, std::bind(&Controller::on_timer_pub_goal_poses, this));

  sub_detection_tracked_ = create_subscription<TrackedFrame>(
    "detection_tracked", 10, std::bind(&Controller::callback_detection_tracked, this, _1));
  sub_geometry_ = create_subscription<GeometryData>(
    "geometry", 10, std::bind(&Controller::callback_geometry, this, _1));
  sub_referee_ = create_subscription<Referee>(
    "referee", 10, std::bind(&Controller::callback_referee, this, _1));
  sub_parsed_referee_ = create_subscription<ParsedReferee>(
    "parsed_referee", 10, std::bind(&Controller::callback_parsed_referee, this, _1));
  sub_named_targets_ = create_subscription<NamedTargets>(
    "named_targets", 10, std::bind(&Controller::callback_named_targets, this, _1));

  auto param_change_callback =
    [this](std::vector<rclcpp::Parameter> parameters) {
      // ROSパラメータの更新
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;
      for (const auto & parameter : parameters) {
        if (parameter.get_name() == "max_acceleration_xy") {
          max_acceleration_xy_ = get_parameter("max_acceleration_xy").get_value<double>();
          RCLCPP_INFO(this->get_logger(), "Update max_acceleration_xy.");
        } else if (parameter.get_name() == "max_acceleration_theta") {
          max_acceleration_theta_ = get_parameter("max_acceleration_theta").get_value<double>();
          RCLCPP_INFO(this->get_logger(), "Update max_acceleration_theta.");
        } else if (parameter.get_name() == "max_velocity_xy") {
          max_velocity_xy_ = get_parameter("max_velocity_xy").get_value<double>();
          RCLCPP_INFO(this->get_logger(), "Update max_velocity_xy.");
        } else if (parameter.get_name() == "max_velocity_theta") {
          max_velocity_theta_ = get_parameter("max_velocity_theta").get_value<double>();
          RCLCPP_INFO(this->get_logger(), "Update max_velocity_theta.");
        } else if (parameter.get_name() == "control_range_xy") {
          param_control_range_xy_ = get_parameter("control_range_xy").get_value<double>();
          RCLCPP_INFO(this->get_logger(), "Update control_range_xy.");
        } else if (parameter.get_name() == "control_a_xy") {
          param_control_a_xy_ = get_parameter("control_a_xy").get_value<double>();
          RCLCPP_INFO(this->get_logger(), "Update control_a_xy.");
        } else if (parameter.get_name() == "control_a_theta") {
          param_control_a_theta_ = get_parameter("control_a_theta").get_value<double>();
          RCLCPP_INFO(this->get_logger(), "Update control_a_theta.");
        } else if (parameter.get_name() == PARAM_THRESHOLD_LOOKING_BALL_DISTANCE) {
          parser_.param_threshold_looking_ball_distance = get_parameter(
            PARAM_THRESHOLD_LOOKING_BALL_DISTANCE).get_value<double>();
          RCLCPP_INFO(
            this->get_logger(), "Update %s",
            PARAM_THRESHOLD_LOOKING_BALL_DISTANCE);
        } else if (parameter.get_name() == PARAM_THRESHOLD_LOOKING_BALL_THETA) {
          parser_.param_threshold_looking_ball_theta = get_parameter(
            PARAM_THRESHOLD_LOOKING_BALL_THETA).get_value<double>();
          RCLCPP_INFO(this->get_logger(), "Update %s", PARAM_THRESHOLD_LOOKING_BALL_THETA);
        } else if (parameter.get_name() == PARAM_THRESHOLD_CAN_DRIBBLE_DISTANCE) {
          parser_.param_can_dribble_distance = get_parameter(
            PARAM_THRESHOLD_CAN_DRIBBLE_DISTANCE).get_value<double>();
          RCLCPP_INFO(
            this->get_logger(), "Update %s",
            PARAM_THRESHOLD_CAN_DRIBBLE_DISTANCE);
        } else if (parameter.get_name() == PARAM_THRESHOLD_CAN_SHOOT_THETA) {
          parser_.param_can_shoot_theta = get_parameter(
            PARAM_THRESHOLD_CAN_SHOOT_THETA).get_value<double>();
          RCLCPP_INFO(this->get_logger(), "Update %s", PARAM_THRESHOLD_CAN_SHOOT_THETA);
        } else if (parameter.get_name() == PARAM_DISTANCE_TO_LOOK_BALL) {
          parser_.param_distance_to_look_ball = get_parameter(
            PARAM_DISTANCE_TO_LOOK_BALL).get_value<double>();
          RCLCPP_INFO(this->get_logger(), "Update %s", PARAM_DISTANCE_TO_LOOK_BALL);
        } else if (parameter.get_name() == PARAM_DISTANCE_TO_ROTATE) {
          parser_.param_distance_to_rotate = get_parameter(
            PARAM_DISTANCE_TO_ROTATE).get_value<double>();
          RCLCPP_INFO(this->get_logger(), "Update %s", PARAM_DISTANCE_TO_ROTATE);
        }
      }
      return result;
    };
  handler_change_param_ = add_on_set_parameters_callback(param_change_callback);
}

void Controller::on_timer_pub_control_command(const unsigned int robot_id)
{
  // 制御器を更新し、コマンドをpublishするタイマーコールバック関数

  // 制御が許可されていない場合は、このタイマーを止めて、停止コマンドタイマーを起動する
  if (control_enable_[robot_id] == false) {
    timer_pub_control_command_[robot_id]->cancel();
    timer_pub_stop_command_[robot_id]->reset();
    return;
  }

  auto command_msg = std::make_unique<RobotCommand>();
  command_msg->robot_id = robot_id;
  command_msg->team_is_yellow = team_is_yellow_;

  // 制御するロボットの情報を得る
  // ロボットの情報が存在しなければ制御を終える
  TrackedRobot my_robot;
  if (!parser_.extract_robot(robot_id, team_is_yellow_, my_robot)) {
    std::string error_msg = "Failed to extract ID:" + std::to_string(robot_id) +
      " robot from detection_tracked msg.";
    RCLCPP_WARN(this->get_logger(), error_msg.c_str());

    switch_to_stop_control_mode(robot_id, false, error_msg);
    return;
  }

  // 目標値を取得する
  // 目標値を取得できなければ速度0を目標値とする
  State goal_pose;
  State final_goal_pose;
  double kick_power = 0.0;
  double dribble_power = 0.0;
  State world_vel;
  auto current_time = steady_clock_.now();
  auto duration = current_time - last_update_time_[robot_id];

  if (parser_.parse_goal(
      goal_handle_[robot_id]->get_goal(), my_robot, goal_pose, final_goal_pose, kick_power,
      dribble_power))
  {
    // field_info_parserの衝突回避を無効かする場合は、下記の行をコメントアウトすること
    goal_pose = parser_.modify_goal_pose_to_avoid_obstacles(
      goal_handle_[robot_id]->get_goal(), my_robot, goal_pose, final_goal_pose);

    // 障害物情報を取得
    auto obstacle_environments = parser_.get_obstacle_environment(
      goal_handle_[robot_id]->get_goal(), my_robot);

    // 現在位置: my_robot.pos
    // 現在速度: my_robot.vel[0]  // optionalなのでvectorに格納している
    // 最終目標位置: goal_pose
    // 障害物情報: obstacle_environments
    // move_with_avoid(my_robot.pos, my_robot.vel[0], goal_pose, obstacle_environments)

    // 目標位置と現在位置の差分
    double diff_x = goal_pose.x - my_robot.pos.x;
    double diff_y = goal_pose.y - my_robot.pos.y;
    double diff_theta = tools::normalize_theta(
      goal_pose.theta -
      my_robot.orientation);

    // tanhに反応する区間の係数
    // double range_xy = 1.0;
    // double range_theta = 0.1;
    // 最大速度調整用の係数(a < 1)
    // double a_xy = 1.0;
    // double a_theta = 0.5;

    // tanh関数を用いた速度制御
    world_vel.x = ctools::velocity_contol_tanh(
      diff_x, param_control_range_xy_,
      param_control_a_xy_ * max_velocity_xy_);
    world_vel.y = ctools::velocity_contol_tanh(
      diff_y, param_control_range_xy_,
      param_control_a_xy_ * max_velocity_xy_);
    // sin関数を用いた角速度制御
    world_vel.theta = ctools::angular_velocity_contol_sin(
      diff_theta,
      param_control_a_theta_ *
      max_velocity_theta_);
  }

  // 最大加速度リミットを適用
  world_vel = limit_world_acceleration(world_vel, last_world_vel_[robot_id], duration);
  // 最大速度リミットを適用
  auto max_velocity_xy = max_velocity_xy_;
  // 最大速度リミットを上書きできる
  if (goal_handle_[robot_id]->get_goal()->max_velocity_xy.size() > 0) {
    max_velocity_xy = std::min(
      goal_handle_[robot_id]->get_goal()->max_velocity_xy[0], max_velocity_xy_);
  }
  world_vel = limit_world_velocity(world_vel, max_velocity_xy);

  // ワールド座標系でのxy速度をロボット座標系に変換
  command_msg->velocity_x = std::cos(my_robot.orientation) * world_vel.x + std::sin(
    my_robot.orientation) * world_vel.y;
  command_msg->velocity_y = -std::sin(my_robot.orientation) * world_vel.x + std::cos(
    my_robot.orientation) * world_vel.y;
  command_msg->velocity_theta = world_vel.theta;

  // キックパワー、ドリブルパワーをセット
  command_msg->kick_power = kick_power;
  command_msg->dribble_power = dribble_power;

  // 制御値を出力する
  pub_command_[robot_id]->publish(std::move(command_msg));

  // 制御更新時間と速度を保存する
  last_update_time_[robot_id] = current_time;
  last_world_vel_[robot_id] = world_vel;

  // ビジュアライズ用に、目標姿勢と最終目標姿勢を出力する
  GoalPose goal_pose_msg;
  goal_pose_msg.robot_id = robot_id;
  goal_pose_msg.team_is_yellow = team_is_yellow_;
  goal_pose_msg.pose = goal_pose;
  goal_poses_map_[robot_id] = goal_pose_msg;

  GoalPose final_goal_pose_msg;
  final_goal_pose_msg.robot_id = robot_id;
  final_goal_pose_msg.team_is_yellow = team_is_yellow_;
  final_goal_pose_msg.pose = final_goal_pose;
  final_goal_poses_map_[robot_id] = final_goal_pose_msg;

  // 途中経過を報告する
  if (need_response_[robot_id]) {
    auto feedback = std::make_shared<RobotControl::Feedback>();
    feedback->remaining_pose.x = goal_pose.x - my_robot.pos.x;
    feedback->remaining_pose.y = goal_pose.y - my_robot.pos.y;
    feedback->remaining_pose.theta = tools::normalize_theta(goal_pose.theta - my_robot.orientation);
    if (my_robot.vel.size() > 0 && my_robot.vel_angular.size() > 0) {
      feedback->present_velocity.x = my_robot.vel[0].x;
      feedback->present_velocity.y = my_robot.vel[0].y;
      feedback->present_velocity.theta = my_robot.vel_angular[0];
    }

    goal_handle_[robot_id]->publish_feedback(feedback);
  }

  if (arrived(my_robot, goal_pose)) {
    // アクションクライアントへの応答が必要な場合は、
    // 目標値に到達した後に制御完了応答を返し、
    // 速度指令値を0にする
    if (need_response_[robot_id]) {
      switch_to_stop_control_mode(robot_id, true, "目的地に到着しました");
    }
  }
}

void Controller::on_timer_pub_stop_command(const unsigned int robot_id)
{
  // 停止コマンドをpublishするタイマーコールバック関数
  // 通信帯域を圧迫しないため、この関数は低周期（例:1s）で実行すること
  auto command_msg = std::make_unique<RobotCommand>();
  command_msg->robot_id = robot_id;
  command_msg->team_is_yellow = team_is_yellow_;

  last_update_time_[robot_id] = steady_clock_.now();
  pub_command_[robot_id]->publish(std::move(command_msg));

  // 制御が許可されたらこのタイマーを止めて、制御タイマーを起動する
  if (control_enable_[robot_id] == true) {
    timer_pub_stop_command_[robot_id]->cancel();
    timer_pub_control_command_[robot_id]->reset();
  }
}

void Controller::on_timer_pub_goal_poses()
{
  // goal_posesをpublishするタイマーコールバック関数
  auto goal_poses = std::make_unique<GoalPoses>();
  auto final_goal_poses = std::make_unique<GoalPoses>();
  for (const auto & robot_id : parser_.active_robot_id_list(team_is_yellow_)) {
    if (goal_poses_map_.count(robot_id) > 0) {
      goal_poses->poses.push_back(goal_poses_map_[robot_id]);
    }
    if (final_goal_poses_map_.count(robot_id) > 0) {
      final_goal_poses->poses.push_back(final_goal_poses_map_[robot_id]);
    }
  }

  vis_data_handler_->publish_vis_goal(std::move(goal_poses), std::move(final_goal_poses));
  // pub_goal_poses_->publish(std::move(goal_poses));
  // pub_final_goal_poses_->publish(std::move(final_goal_poses));
}

void Controller::callback_detection_tracked(const TrackedFrame::SharedPtr msg)
{
  parser_.set_detection_tracked(msg);
}

void Controller::callback_geometry(const GeometryData::SharedPtr msg)
{
  parser_.set_geometry(msg);
}

void Controller::callback_referee(const Referee::SharedPtr msg)
{
  parser_.set_referee(msg);
}

void Controller::callback_parsed_referee(const ParsedReferee::SharedPtr msg)
{
  parser_.set_parsed_referee(msg);
}

void Controller::callback_named_targets(const NamedTargets::SharedPtr msg)
{
  parser_.set_named_targets(msg);
}

rclcpp_action::GoalResponse Controller::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const RobotControl::Goal> goal, const unsigned int robot_id)
{
  (void)uuid;
  (void)robot_id;

  // 制御を停止する場合は無条件でaccept
  if (goal->stop) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // 目標値の解析に失敗したらReject
  State goal_pose;
  if (!parser_.parse_goal(goal, goal_pose)) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Controller::handle_cancel(
  const std::shared_ptr<GoalHandleRobotControl> goal_handle, const unsigned int robot_id)
{
  // キャンセル信号を受け取ったら制御を停止する
  (void)goal_handle;
  control_enable_[robot_id] = false;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void Controller::handle_accepted(
  std::shared_ptr<GoalHandleRobotControl> goal_handle,
  const unsigned int robot_id)
{
  // 目標値が承認されたときに実行するハンドラ
  auto goal = goal_handle->get_goal();
  if (goal->stop) {
    // 制御を停止する
    auto result = std::make_shared<RobotControl::Result>();
    result->success = true;
    result->message = "制御を停止します";
    goal_handle->succeed(result);
    switch_to_stop_control_mode(robot_id, true, "RobotControl.stopがセットされました");
    return;
  }

  if (goal->keep_control) {
    // 目標値に到達しても制御を続ける
    need_response_[robot_id] = false;

    // アクションを完了する
    auto result = std::make_shared<RobotControl::Result>();
    result->success = true;
    result->message = "Success!";
    goal_handle->succeed(result);
  } else {
    need_response_[robot_id] = true;
  }

  control_enable_[robot_id] = true;
  goal_handle_[robot_id] = goal_handle;
}

State Controller::limit_world_velocity(
  const State & velocity, const double & max_velocity_xy) const
{
  // ワールド座標系のロボット速度に制限を掛ける
  auto velocity_norm = std::hypot(velocity.x, velocity.y);
  auto velocity_ratio = velocity_norm / max_velocity_xy;

  State modified_velocity = velocity;
  if (velocity_ratio > 1.0) {
    modified_velocity.x /= velocity_ratio;
    modified_velocity.y /= velocity_ratio;
  }
  modified_velocity.theta = std::clamp(velocity.theta, -max_velocity_theta_, max_velocity_theta_);

  return modified_velocity;
}

State Controller::limit_world_acceleration(
  const State & velocity, const State & last_velocity,
  const rclcpp::Duration & dt) const
{
  // ワールド座標系のロボット加速度に制限を掛ける
  State acc;
  acc.x = (velocity.x - last_velocity.x) / dt.seconds();
  acc.y = (velocity.y - last_velocity.y) / dt.seconds();
  acc.theta = (velocity.theta - last_velocity.theta) / dt.seconds();

  auto acc_norm = std::hypot(acc.x, acc.y);
  auto acc_ratio = acc_norm / max_acceleration_xy_;

  if (acc_ratio > 1.0) {
    acc.x /= acc_ratio;
    acc.y /= acc_ratio;
  }
  acc.theta = std::clamp(acc.theta, -max_acceleration_theta_, max_acceleration_theta_);

  State modified_velocity = velocity;
  modified_velocity.x = last_velocity.x + acc.x * dt.seconds();
  modified_velocity.y = last_velocity.y + acc.y * dt.seconds();
  modified_velocity.theta = last_velocity.theta + acc.theta * dt.seconds();
  return modified_velocity;
}

bool Controller::arrived(const TrackedRobot & my_robot, const State & goal_pose)
{
  // 目的地に到着したかどうか判定する
  // my_robotが速度データを持っていたら、速度が一定値以下に低下していることも判定する
  const double DISTANCE_THRESHOLD = 0.01;  // meters
  const double THETA_THRESHOLD = 2.0 * M_PI / 180.0;  // radians
  const double VELOCITY_THRESHOLD = 0.1;  // m/s
  const double OMEGA_THRESHOLD = 0.1 * M_PI;  // rad/s


  // 速度が小さくなっているか判定
  if (my_robot.vel.size() > 0 && my_robot.vel_angular.size() > 0) {
    if (std::fabs(my_robot.vel[0].x) > VELOCITY_THRESHOLD ||
      std::fabs(my_robot.vel[0].y) > VELOCITY_THRESHOLD ||
      std::fabs(my_robot.vel_angular[0]) > OMEGA_THRESHOLD)
    {
      return false;
    }
  }

  // 直線距離の差分が小さくなっているか判定
  double diff_x = goal_pose.x - my_robot.pos.x;
  double diff_y = goal_pose.y - my_robot.pos.y;
  double remaining_theta =
    std::fabs(tools::normalize_theta(goal_pose.theta - my_robot.orientation));
  double remaining_distance = std::hypot(diff_x, diff_y);
  if (remaining_distance < DISTANCE_THRESHOLD && remaining_theta < THETA_THRESHOLD) {
    return true;
  }
  return false;
}


bool Controller::switch_to_stop_control_mode(
  const unsigned int robot_id, const bool success, const std::string & error_msg)
{
  // 指定されたIDのロボットの制御を終了し、停止制御モードに切り替える

  if (robot_id >= ROBOT_NUM) {
    RCLCPP_WARN(this->get_logger(), "無効なロボットID: %d(>=%d)です", robot_id, ROBOT_NUM);
    return false;
  }

  // アクションサーバが動いているときは、応答を返す
  if (need_response_[robot_id]) {
    auto result = std::make_shared<RobotControl::Result>();
    result->success = success;
    result->message = error_msg;
    goal_handle_[robot_id]->abort(result);
  }

  // 制御を禁止
  control_enable_[robot_id] = false;
  // アクションの応答フラグを無効化
  need_response_[robot_id] = false;
  // 制御タイマーを停止
  timer_pub_control_command_[robot_id]->cancel();
  // ストップ信号タイマーを最下位
  timer_pub_stop_command_[robot_id]->reset();
  // 停止コマンドを送信する
  auto stop_command_msg = std::make_unique<RobotCommand>();
  stop_command_msg->robot_id = robot_id;
  stop_command_msg->team_is_yellow = team_is_yellow_;
  pub_command_[robot_id]->publish(std::move(stop_command_msg));

  return true;
}

}  // namespace consai_robot_controller

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(consai_robot_controller::Controller)
