#include "consai_robot_controller/trajectory_follow_control.hpp"
#include <cstdint>

// TrajectoryFollowController クラスの定義
TrajectoryFollowController::TrajectoryFollowController() {
    this->state_ = ControllerState::INITIALIZED;
    this->tracked_time_ = 0;
    this->kp_ = 1.0;
    this->trajectory_ = nullptr;
}

TrajectoryFollowController::TrajectoryFollowController(_Float64 kp, double dt) {
    this->state_ = ControllerState::INITIALIZED;
    this->tracked_time_ = 0;
    this->kp_ = kp;
    this->dt_ = dt;
    this->latest_target_state_ = State2D();
    this->trajectory_ = nullptr;
}


void TrajectoryFollowController::initialize(std::shared_ptr<BangBangTrajectory2D> trajectory) {
    this->trajectory_ = trajectory;
    this->state_ = ControllerState::INITIALIZED;
    this->latest_target_state_ = State2D();
    this->tracked_time_ = 0;
}

std::pair<Velocity2D, TrajectoryFollowController::ControllerState> TrajectoryFollowController::run(const State2D& current_state) {
    /** 
     * 軌道追従の制御を行う制御器
     * 現在位置と次ステップの目標軌道位置との差分を元にPID制御を行う。
     * また、フィードフォワード項として、現ステップと次ステップの目標位置の差分(=理想の速度)を使用する。
     */
    Velocity2D output = Velocity2D();

    // 状態の判定
    ControllerState state = ControllerState::RUNNING;

    if (this->state_ == ControllerState::INITIALIZED) {
        this->state_ = ControllerState::RUNNING;
    }

    // 前ステップの目標位置の取得
    Vector2D last_target_pose = this->trajectory_->get_position(this->tracked_time_);
    Pose2D last_target_pose2(last_target_pose.x, last_target_pose.y, 0.0);

    // 今ステップの目標位置の取得
    Vector2D target_pose = this->trajectory_->get_position(this->tracked_time_ + this->dt_);
    Pose2D target_pose2(target_pose.x, target_pose.y, 0.0);

    // 今ステップの目標速度の取得
    Vector2D target_velocity = this->trajectory_->get_velocity(this->tracked_time_ + this->dt_);

    // x方向の制御
    output.x = this->control(current_state.pose.x, target_pose.x, target_velocity.x);

    // y方向の制御
    output.y = this->control(current_state.pose.y, target_pose.y, target_velocity.y);

    // theta方向の制御
    // output.theta = this->control(current_state.pose.theta, target_pose.theta, last_target_pose.theta);
    // TODO: implement theta control

    // 時間の更新
    this->tracked_time_ += this->dt_;

    this->latest_target_state_ = State2D(target_pose2, Velocity2D(target_velocity.x, target_velocity.y, 0.0));  // TODO: implement theta

    // 終了判定
    if (this->tracked_time_ >= this->trajectory_->get_total_time()) {
        state = ControllerState::COMPLETE;
    }

    return std::make_pair(output, state);
}

double TrajectoryFollowController::control(double current_position, double target_position, double target_velocity) {
    double error = target_position - current_position;
    double output = kp_ * error + target_velocity;

    return output;
}


// TrajectoryVisualizer クラスの定義
consai_visualizer_msgs::msg::Objects TrajectoryVisualizer::createObjectsFromTrajectory(BangBangTrajectory2D& trajectory) {
    consai_visualizer_msgs::msg::Objects objects;

    objects.layer = "trajectory";
    objects.sub_layer = "trajectory";

    double dt = 0.1;
    for (double t = 0; t < trajectory.get_total_time(); t += dt) {
        consai_visualizer_msgs::msg::ShapeLine line;

        line.p1.x = trajectory.get_position(t).x;
        line.p1.y = trajectory.get_position(t).y;
        line.p2.x = trajectory.get_position(t + dt).x;
        line.p2.y = trajectory.get_position(t + dt).y;
        line.size = 1;
        line.color.red = 0.0;
        line.color.green = 0.0;
        line.color.blue = 1.0;
        line.color.alpha = 1.0;
        objects.lines.push_back(line);
    }

    return objects;
}