#pragma once

#include "rclcpp/rclcpp.hpp"
#include "consai_visualizer_msgs/msg/objects.hpp"
#include "consai_visualizer_msgs/msg/shape_line.hpp"
#include "consai_robot_controller/trajectory/bangbangtrajectory3d.h"


/**
 * @brief Trajectoryを追従するためのクラス
 * @details
 * Trajectoryを追従するためのクラス
*/
class TrajectoryFollowController
{
public:
  /**
   * @brief コントローラの状態
   * @details
   * コントローラの状態
  */
  enum ControllerState
  {
    INITIALIZED,
    RUNNING,
    COMPLETE,
    FAILED
  };

  State2D latest_target_state_;

  TrajectoryFollowController();
  TrajectoryFollowController(
    _Float64 kp_linear, _Float64 kd_linear, _Float64 kp_angular_,
    _Float64 kd_angular, double dt);

  /**
   * @brief コントローラの初期化
   * @param trajectory 追従するTrajectory
   */
  void initialize(std::shared_ptr<BangBangTrajectory3D> trajectory);

  /**
   * @brief 現在の状態を元に次ステップの指令速度とコントローラのステートを計算する
   * @param current_state 現在の状態
   * @return 次ステップの指令速度とコントローラのステート
   */
  std::pair<Velocity2D, ControllerState> run(const State2D & current_state);

private:
  enum ControllerMode
  {
    FF_AND_P,
    P
  };

  double controlLinear(
    ControllerMode mode, double current_position, double target_position,
    double target_velocity);
  double controlAngular(double current_position, double target_position, double target_velocity);

  std::shared_ptr<BangBangTrajectory3D> trajectory_;
  ControllerState state_;

  _Float64 kp_linear_ = 10.0;
  // _Float64  ki_linear_ = 0.0;
  _Float64 kd_linear_ = 0.0;

  _Float64 kp_angular_ = 10.0;
  // _Float64  ki_angular_ = 0.0;
  _Float64 kd_angular_ = 0.0;

  double tracked_time_ = 0;  // 追従制御開始時刻からの経過時刻
  double dt_ = 0.0;          // 制御周期
  double last_error_linear_ = 0;
};


/**
 * Visualize 関連
*/
class TrajectoryVisualizer
{
public:
  /**
       * @brief TrajectoryからObjetcsを生成するクラスメソッド
       * @param trajectory
       * @return Objects
       */
  static consai_visualizer_msgs::msg::Objects createObjectsFromTrajectory(
    BangBangTrajectory2D & trajectory);
};
