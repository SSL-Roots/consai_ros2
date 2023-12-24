#include "consai_robot_controller/trajectory_follow_control.hpp"
#include <cstdint>

// Pose2D クラスの定義
Pose2D::Pose2D() {
    this->x = 0.0;
    this->y = 0.0;
    this->theta = 0.0;
}

Pose2D::Pose2D(double x, double y, double theta) {
    this->x = x;
    this->y = y;
    this->theta = theta;
}

// Velocity2D クラスの定義
Velocity2D::Velocity2D() {
    this->x = 0.0;
    this->y = 0.0;
    this->theta = 0.0;
}

Velocity2D::Velocity2D(double x, double y, double theta) {
    this->x = x;
    this->y = y;
    this->theta = theta;
}

// State2D クラスの定義
State2D::State2D() {
    this->pose = Pose2D();
    this->velocity = Velocity2D();
}

State2D::State2D(Pose2D pose, Velocity2D velocity) {
    this->pose = pose;
    this->velocity = velocity;
}

// TimeStamp クラスの定義
TimeStamp::TimeStamp() {
    this->start_time_ms = 0;
    this->timestamp_ms = 0;
}

TimeStamp::TimeStamp(uint64_t start_time_ms, uint64_t timestamp_ms) {
    this->start_time_ms = start_time_ms;
    this->timestamp_ms = timestamp_ms;
}

// Pose2DStamped クラスの定義
Pose2DStamped::Pose2DStamped() {
    this->pose = Pose2D();
    this->timestamp = TimeStamp();
}

Pose2DStamped::Pose2DStamped(Pose2D pose, TimeStamp timestamp) {
    this->pose = pose;
    this->timestamp = timestamp;
}

// Trajectory クラスの定義
Trajectory::Trajectory() {
    this->poses = std::vector<Pose2D>();
    this->step_time_ms = 0;
}

Trajectory::Trajectory(std::vector<Pose2D> poses, uint64_t dt_ms) {
    this->step_time_ms = dt_ms;
    this->poses = poses;
}

/**
 * 指定した時刻の軌道位置を取得する
 * 軌道のポイントの間にある時刻の場合は、線形補間を行う
 * @param time_ms 時刻
 * @return 指定した時刻の軌道位置
*/
Pose2D Trajectory::getPoseAtTime(uint64_t time_ms) {
    // 軌道のポイントの間にある時刻の場合は、線形補間を行う
    uint64_t index = time_ms / this->step_time_ms;
    if (index < this->poses.size() - 1) {
        double ratio = (time_ms % this->step_time_ms) / (double)this->step_time_ms;
        double x = this->poses[index].x + (this->poses[index + 1].x - this->poses[index].x) * ratio;
        double y = this->poses[index].y + (this->poses[index + 1].y - this->poses[index].y) * ratio;
        double theta = this->poses[index].theta + (this->poses[index + 1].theta - this->poses[index].theta) * ratio;
        return Pose2D(x, y, theta);
    } else {
        return this->poses[this->poses.size() - 1];
    }
}

// TrajectoryFollowController クラスの定義
TrajectoryFollowController::TrajectoryFollowController() {
    this->trajectory_ = Trajectory();
    this->state_ = ControllerState::INITIALIZED;
    this->tracked_time_ms_ = 0;
    this->kp_ = 1.0;
}

void TrajectoryFollowController::initialize(const Trajectory& trajectory) {
    this->trajectory_ = trajectory;
    this->state_ = ControllerState::INITIALIZED;
    this->tracked_time_ms_ = 0;
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
    Pose2D last_target_pose = this->trajectory_.getPoseAtTime(this->tracked_time_ms_);

    // 今ステップの目標位置の取得
    Pose2D target_pose = this->trajectory_.getPoseAtTime(this->tracked_time_ms_ + this->dt_ms_);

    // x方向の制御
    output.x = this->control(current_state.pose.x, target_pose.x, last_target_pose.x);

    // y方向の制御
    output.y = this->control(current_state.pose.y, target_pose.y, last_target_pose.y);

    // theta方向の制御
    output.theta = this->control(current_state.pose.theta, target_pose.theta, last_target_pose.theta);

    // 時間の更新
    this->tracked_time_ms_ += this->dt_ms_;

    // 終了判定
    if (this->tracked_time_ms_ >= this->trajectory_.poses.size() * this->dt_ms_) {
        state = ControllerState::COMPLETE;
    }

    return std::make_pair(output, state);
}

double TrajectoryFollowController::control(double current, double target, double current_target) {
    double error = target - current;
    double ideal_speed = (target - current_target) / this->dt_ms_;
    double output = kp_ * error + ideal_speed;
    return output;
}


// TrajectoryVisualizer クラスの定義
consai_visualizer_msgs::msg::Objects TrajectoryVisualizer::createObjectsFromTrajectory(const Trajectory& trajectory) {
    consai_visualizer_msgs::msg::Objects objects;

    objects.layer = "trajectory";
    objects.sub_layer = "trajectory";

    // trajectory is visualised by objects.lines
    for (auto i = 0; i < trajectory.poses.size() - 1; i++) {
        consai_visualizer_msgs::msg::ShapeLine line;

        line.p1.x = trajectory.poses[i].x;
        line.p1.y = trajectory.poses[i].y;
        line.p2.x = trajectory.poses[i + 1].x;
        line.p2.y = trajectory.poses[i + 1].y;
        line.size = 1;
        line.color.red = 1.0;
        line.color.green = 0.0;
        line.color.blue = 0.0;
        line.color.alpha = 1.0;
        objects.lines.push_back(line);
    }

    return objects;
}