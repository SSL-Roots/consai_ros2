#pragma once

#include "rclcpp/rclcpp.hpp"
#include "consai_visualizer_msgs/msg/objects.hpp"
#include "consai_visualizer_msgs/msg/shape_line.hpp"


class Pose2D {
public:
  double x;
  double y;
  double theta;

  Pose2D();
  Pose2D(double x, double y, double theta);
};

class Velocity2D {
public:
  double x;
  double y;
  double theta;

  Velocity2D();
  Velocity2D(double x, double y, double theta);
};

class State2D {
public:
  Pose2D pose;
  Velocity2D velocity;

  State2D();
  State2D(Pose2D pose, Velocity2D velocity);
};

class Trajectory {
public:
  Trajectory();
  Trajectory(std::vector<Pose2D> poses, double dt);
  Pose2D getPoseAtTime(double sec);
    
  std::vector<Pose2D> poses;

private:
  double dt_;
};

/**
 * @brief Trajectoryを追従するためのクラス
 * @details
 * Trajectoryを追従するためのクラス
*/
class TrajectoryFollowController {
public:
  /**
   * @brief コントローラの状態
   * @details
   * コントローラの状態
  */
  enum ControllerState {
    INITIALIZED,
    RUNNING,
    COMPLETE,
    FAILED
  };

  TrajectoryFollowController();
  TrajectoryFollowController(_Float64 kp, double dt);

  /**
   * @brief コントローラの初期化
   * @param trajectory 追従するTrajectory
   */
  void initialize(std::shared_ptr<Trajectory> trajectory);

  /**
   * @brief 現在の状態を元に次ステップの指令速度とコントローラのステートを計算する
   * @param current_state 現在の状態
   * @return 次ステップの指令速度とコントローラのステート
   */
  std::pair<Velocity2D, ControllerState> run(const State2D& current_state);

private:
  double control(double current, double target, double current_target);

  std::shared_ptr<Trajectory> trajectory_;
  ControllerState state_;

  _Float64  kp_ = 10.0;
  // _Float64  ki_ = 0.0;
  // _Float64  kd_ = 0.0;

  double tracked_time_ = 0;  // 追従制御開始時刻からの経過時刻
  double dt_ = 0.0;          // 制御周期
};



/**
 * Visualize 関連
*/
class TrajectoryVisualizer {
    public:
        /**
         * @brief TrajectoryからObjetcsを生成するクラスメソッド
         * @param trajectory
         * @return Objects
         */ 
        static consai_visualizer_msgs::msg::Objects createObjectsFromTrajectory(const Trajectory& trajectory);
};
