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
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "consai_robot_controller/controller_component.hpp"
#include "consai_robot_controller/tools/control_tools.hpp"
#include "consai_robot_controller/global_for_debug.hpp"
#include "consai_tools/geometry_tools.hpp"
#include "rclcpp/rclcpp.hpp"

namespace consai_robot_controller
{

namespace tools = geometry_tools;
namespace ctools = control_tools;

const int ROBOT_NUM = 16;

Controller::Controller(const rclcpp::NodeOptions & options)
: Node("controller", options)
{
  using namespace std::placeholders;

  declare_parameter("team_is_yellow", false);
  declare_parameter("invert", false);
  declare_parameter("hard_limit_acceleration_xy", 2.0);
  declare_parameter("soft_limit_acceleration_xy", 2.0);
  declare_parameter("hard_limit_acceleration_theta", 2.0 * M_PI);
  declare_parameter("soft_limit_acceleration_theta", 2.0 * M_PI);
  declare_parameter("hard_limit_velocity_xy", 2.0);
  declare_parameter("soft_limit_velocity_xy", 2.0);
  declare_parameter("hard_limit_velocity_theta", 2.0 * M_PI);
  declare_parameter("soft_limit_velocity_theta", 2.0 * M_PI);
  declare_parameter("control_range_xy", 1.0);
  declare_parameter("control_a_xy", 1.0);
  declare_parameter("control_a_theta", 0.5);
  declare_parameter("p_gain_xy", 1.5);
  declare_parameter("d_gain_xy", 0.0);
  declare_parameter("p_gain_theta", 2.5);
  declare_parameter("d_gain_theta", 0.0);
  declare_parameter("delayfactor_sec", 0.1);

  const auto visibility_threshold = 0.01;
  detection_extractor_ = std::make_shared<parser::DetectionExtractor>(visibility_threshold);
  parser_ = std::make_shared<FieldInfoParser>(
    get_parameter("team_is_yellow").get_value<bool>(),
    get_parameter("invert").get_value<bool>(),
    detection_extractor_);
  obstacle_observer_ = std::make_shared<obstacle::ObstacleObserver>(detection_extractor_);

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

    robot_control_map_[i] = std::make_shared<RobotControlMsg>();
    auto robot_control_callback = [this](
      const RobotControlMsg::SharedPtr msg, const unsigned int robot_id) {
        this->robot_control_map_[robot_id] = msg;
      };
    // Can not use auto. Ref: https://github.com/ros2/rclcpp/issues/273
    std::function<void(const RobotControlMsg::SharedPtr msg)> fcn = std::bind(
      robot_control_callback, _1, i);

    pub_current_pose_.push_back(
      create_publisher<State>(
        "robot" + std::to_string(i) + "/current_pose", 10)
    );

    pub_current_vel_.push_back(
      create_publisher<State>(
        "robot" + std::to_string(i) + "/current_vel", 10)
    );

    pub_goal_pose_.push_back(
      create_publisher<State>(
        "robot" + std::to_string(i) + "/goal_pose", 10)
    );

    pub_target_speed_world_.push_back(
      create_publisher<State>(
        "robot" + std::to_string(i) + "/target_speed_world", 10)
    );

    pub_control_output_ff_.push_back(
      create_publisher<State>(
        "robot" + std::to_string(i) + "/control_output_ff", 10)
    );

    pub_control_output_p_.push_back(
      create_publisher<State>(
        "robot" + std::to_string(i) + "/control_output_p", 10)
    );

    std::string name_space = team_color + std::to_string(i);
    sub_robot_control_.push_back(
      create_subscription<RobotControlMsg>(
        name_space + "/control", 10, fcn));

    last_update_time_.push_back(steady_clock_.now());

    // bindでは関数を宣言できなかったので、ラムダ式を使用する
    // Ref: https://github.com/ros2/rclcpp/issues/273#issuecomment-263826519
    timer_pub_control_command_.push_back(
      create_wall_timer(
        control_loop_cycle_, [this, robot_id = i]() {this->on_timer_pub_control_command(robot_id);}
      )
    );

    locomotion_controller_.push_back(
      LocomotionController(i, control_loop_cycle_.count() / 1000.0)
    );

    last_world_vel_.push_back(State());
  }

  pub_goal_poses_ = create_publisher<GoalPoses>("goal_poses", 10);
  pub_final_goal_poses_ = create_publisher<GoalPoses>("final_goal_poses", 10);
  vis_data_handler_ = std::make_shared<VisualizationDataHandler>(
    create_publisher<VisualizerObjects>(
      "visualizer_objects", rclcpp::SensorDataQoS()));
  timer_pub_goal_poses_ =
    create_wall_timer(10ms, std::bind(&Controller::on_timer_pub_goal_poses, this));

  auto detection_callback = [this](const TrackedFrame::SharedPtr msg) {
      parser_->set_detection_tracked(msg);
    };
  sub_detection_tracked_ = create_subscription<TrackedFrame>(
    "detection_tracked", 10, detection_callback);

  auto geometry_callback = [this](const GeometryData::SharedPtr msg) {
      parser_->set_geometry(msg);
    };
  sub_geometry_ = create_subscription<GeometryData>(
    "geometry", 10, geometry_callback);

  auto referee_callback = [this](const Referee::SharedPtr msg) {
      parser_->set_referee(msg);
    };
  sub_referee_ = create_subscription<Referee>(
    "referee", 10, referee_callback);

  auto parsed_referee_callback = [this](const ParsedReferee::SharedPtr msg) {
      parser_->set_parsed_referee(msg);
    };
  sub_parsed_referee_ = create_subscription<ParsedReferee>(
    "parsed_referee", 10, parsed_referee_callback);

  auto named_targets_callback = [this](const NamedTargets::SharedPtr msg) {
      parser_->set_named_targets(msg);
    };
  sub_named_targets_ = create_subscription<NamedTargets>(
    "named_targets", 10, named_targets_callback);
}

void Controller::on_timer_pub_control_command(const unsigned int robot_id)
{
  if (robot_control_map_[robot_id]->stop) {
    publish_stop_command(robot_id);
    return;
  }

  if (!parser_->is_parsable(robot_control_map_[robot_id])) {
    publish_stop_command(robot_id);
    return;
  }

  // 制御するロボットの情報を得る
  // ロボットの情報が存在しなければ制御を終える
  TrackedRobot my_robot;
  if (!detection_extractor_->extract_robot(robot_id, team_is_yellow_, my_robot)) {
    std::string error_msg = "Failed to extract ID:" + std::to_string(robot_id) +
      " robot from detection_tracked msg.";
    RCLCPP_WARN(this->get_logger(), error_msg.c_str());

    publish_stop_command(robot_id);
    robot_control_map_[robot_id]->stop = true;
    return;
  }

  // 目標値を取得する
  // 目標値を取得できなければ速度0を目標値とする
  State goal_pose;
  State final_goal_pose;
  double kick_power = 0.0;
  double dribble_power = 0.0;

  const auto current_time = steady_clock_.now();
  const auto duration = current_time - last_update_time_[robot_id];
  if (!parser_->parse_goal(
      robot_control_map_[robot_id], my_robot, goal_pose, final_goal_pose, kick_power,
      dribble_power))
  {
    RCLCPP_WARN(this->get_logger(), "Failed to parse goal of robot_id:%d", robot_id);
    publish_stop_command(robot_id);
    return;
  }

  // field_info_parserの衝突回避を無効化する場合は、下記の行をコメントアウトすること
  goal_pose = parser_->modify_goal_pose_to_avoid_obstacles(
    robot_control_map_[robot_id], my_robot, goal_pose, final_goal_pose);

  State world_vel;

  // 各種パラメータの設定
  auto hard_limit_vel_xy = get_parameter("hard_limit_velocity_xy").as_double();
  auto soft_limit_vel_xy = get_parameter("soft_limit_velocity_xy").as_double();
  auto hard_limit_vel_theta = get_parameter("hard_limit_velocity_theta").as_double();
  auto soft_limit_vel_theta = get_parameter("soft_limit_velocity_theta").as_double();
  auto hard_limit_acc_xy =  get_parameter("hard_limit_acceleration_xy").as_double();
  auto soft_limit_acc_xy =  get_parameter("soft_limit_acceleration_xy").as_double();
  auto hard_limit_acc_theta = get_parameter("hard_limit_acceleration_theta").as_double();
  auto soft_limit_acc_theta = get_parameter("soft_limit_acceleration_theta").as_double();
  const auto control_range_xy = get_parameter("control_range_xy").as_double();
  const auto control_a_xy = get_parameter("control_a_xy").as_double();
  const auto control_a_theta = get_parameter("control_a_theta").as_double();

  // 最大速度が上書きされていたらそちらの値を使う
  if (robot_control_map_[robot_id]->max_velocity_xy.size() > 0) {
    hard_limit_vel_xy = robot_control_map_[robot_id]->max_velocity_xy[0];
  }

  // パラメータが更新された場合は、ロボットの制御器に反映する
  if (locomotion_controller_[robot_id].getHardLimitLinearVelocity() != hard_limit_vel_xy ||
    locomotion_controller_[robot_id].getHardLimitAngularVelocity() != hard_limit_vel_theta ||
    locomotion_controller_[robot_id].getHardLimitLinearAcceleration() != hard_limit_acc_xy ||
    locomotion_controller_[robot_id].getHardLimitAngularAcceleration() != hard_limit_acc_theta)
  {
    locomotion_controller_[robot_id].setParameters(
      get_parameter("p_gain_xy").as_double(),
      get_parameter("d_gain_xy").as_double(),
      get_parameter("p_gain_theta").as_double(),
      get_parameter("d_gain_theta").as_double(),
      hard_limit_vel_xy, hard_limit_vel_theta, hard_limit_acc_xy, hard_limit_acc_theta);

    // 軌道の再生成
    locomotion_controller_[robot_id].moveToPose(
      Pose2D(goal_pose.x, goal_pose.y, goal_pose.theta),
      State2D(
        Pose2D(my_robot.pos.x, my_robot.pos.y, my_robot.orientation),
        Velocity2D(my_robot.vel[0].x, my_robot.vel[0].y, my_robot.vel_angular[0])
      )
    );
  }

  // 前回の目標値と今回が異なる場合にのみmoveToPoseを呼び出す
  Pose2D current_goal_pose = this->locomotion_controller_[robot_id].getGoal();
  if (goal_pose.x != current_goal_pose.x || goal_pose.y != current_goal_pose.y || goal_pose.theta != current_goal_pose.theta)
  {
    this->locomotion_controller_[robot_id].moveToPose(
      Pose2D(goal_pose.x, goal_pose.y, goal_pose.theta),
      State2D(
        Pose2D(my_robot.pos.x, my_robot.pos.y, my_robot.orientation),
        Velocity2D(my_robot.vel[0].x, my_robot.vel[0].y, my_robot.vel_angular[0])
      )
    );
  }

  // 制御の実行
  auto [output_vel, controller_state] = this->locomotion_controller_[robot_id].run(
    State2D(
      Pose2D(my_robot.pos.x, my_robot.pos.y, my_robot.orientation),
      Velocity2D(my_robot.vel[0].x, my_robot.vel[0].y, my_robot.vel_angular[0])
    )
  );

  // 型変換
  world_vel.x = output_vel.x;
  world_vel.y = output_vel.y;
  world_vel.theta = output_vel.theta;

  // sin関数を用いた角速度制御
  double diff_theta = tools::normalize_theta(
    goal_pose.theta -
    my_robot.orientation);
  world_vel.theta = ctools::angular_velocity_contol_sin(
    diff_theta,
    control_a_theta *
    hard_limit_vel_theta);

  // 直進を安定させるため、xy速度が大きいときは、角度制御をしない
  const auto vel_norm = std::hypot(world_vel.x, world_vel.y);
  if (vel_norm > 1.5) {
    world_vel.theta = 0.0;
  }

  auto command_msg = std::make_unique<RobotCommand>();
  command_msg->robot_id = robot_id;
  command_msg->team_is_yellow = team_is_yellow_;

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

  // デバッグ用に出力する
  {
    State2D current_goal = State2D(this->locomotion_controller_[robot_id].getCurrentTargetState());
    State current_goal_state;
    current_goal_state.x = current_goal.pose.x;
    current_goal_state.y = current_goal.pose.y;
    current_goal_state.theta = current_goal.pose.theta;
    pub_goal_pose_[robot_id]->publish(current_goal_state);
    pub_target_speed_world_[robot_id]->publish(world_vel);

    State my_pose;
    my_pose.x = my_robot.pos.x;
    my_pose.y = my_robot.pos.y;
    my_pose.theta = my_robot.orientation;
    pub_current_pose_[robot_id]->publish(my_pose);

    // velが存在するときのみ速度情報をPublish
    if (my_robot.vel.size() > 0 && my_robot.vel_angular.size() > 0) {
      auto my_vel = std::make_unique<State>();
      my_vel->x      = my_robot.vel[0].x;
      my_vel->y      = my_robot.vel[0].y;
      my_vel->theta  = my_robot.vel_angular[0];
      pub_current_vel_[robot_id]->publish(std::move(my_vel));
    }

    State control_output_ff, control_output_p;
    control_output_ff.x = global_for_debug::last_control_output_ff.x;
    control_output_ff.y = global_for_debug::last_control_output_ff.y;
    control_output_ff.theta = global_for_debug::last_control_output_ff.theta;
    control_output_p.x = global_for_debug::last_control_output_p.x;
    control_output_p.y = global_for_debug::last_control_output_p.y;
    control_output_p.theta = global_for_debug::last_control_output_p.theta;
    pub_control_output_ff_[robot_id]->publish(control_output_ff);
    pub_control_output_p_[robot_id]->publish(control_output_p);
  }


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
}

void Controller::on_timer_pub_goal_poses()
{
  // 目標位置・姿勢を描画情報として出力する
  for (const auto & robot_id : detection_extractor_->active_robot_id_list(team_is_yellow_)) {
    TrackedRobot my_robot;
    if (!detection_extractor_->extract_robot(robot_id, team_is_yellow_, my_robot)) {
      continue;
    }

    if (goal_poses_map_.count(robot_id) > 0 && final_goal_poses_map_.count(robot_id) > 0) {
      vis_data_handler_->append_vis_goal(
        my_robot, goal_poses_map_[robot_id], final_goal_poses_map_[robot_id]);
    }
  }

  vis_data_handler_->publish_and_reset_vis_goal();
}

State Controller::limit_world_velocity(
  const State & velocity, const double & max_velocity_xy,
  const double & max_velocity_theta) const
{
  // ワールド座標系のロボット速度に制限を掛ける
  auto velocity_norm = std::hypot(velocity.x, velocity.y);
  auto velocity_ratio = velocity_norm / max_velocity_xy;

  State modified_velocity = velocity;
  if (velocity_ratio > 1.0) {
    modified_velocity.x /= velocity_ratio;
    modified_velocity.y /= velocity_ratio;
  }
  modified_velocity.theta = std::clamp(
    velocity.theta, -max_velocity_theta, max_velocity_theta);

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

  const auto max_acc_xy = get_parameter("hard_limit_acceleration_xy").as_double();
  const auto max_acc_theta = get_parameter("hard_limit_acceleration_theta").as_double();
  const auto acc_norm = std::hypot(acc.x, acc.y);
  const auto acc_ratio = acc_norm / max_acc_xy;

  if (acc_ratio > 1.0) {
    acc.x /= acc_ratio;
    acc.y /= acc_ratio;
  }
  acc.theta = std::clamp(acc.theta, -max_acc_theta, max_acc_theta);

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


bool Controller::publish_stop_command(const unsigned int robot_id)
{
  if (robot_id >= ROBOT_NUM) {
    RCLCPP_WARN(this->get_logger(), "無効なロボットID: %d(>=%d)です", robot_id, ROBOT_NUM);
    return false;
  }

  auto stop_command_msg = std::make_unique<RobotCommand>();
  stop_command_msg->robot_id = robot_id;
  stop_command_msg->team_is_yellow = team_is_yellow_;
  pub_command_[robot_id]->publish(std::move(stop_command_msg));

  return true;
}

}  // namespace consai_robot_controller

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(consai_robot_controller::Controller)
