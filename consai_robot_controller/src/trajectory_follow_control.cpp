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
    this->dt_ms = 0;
}

Trajectory::Trajectory(std::vector<Pose2D> poses, uint64_t dt_ms) {
    this->dt_ms = dt_ms;
    this->poses = poses;
}

// TrajectoryFollowController クラスの定義
TrajectoryFollowController::TrajectoryFollowController() {
    this->trajectory_ = Trajectory();
    this->state_ = ControllerState::INITIALIZED;
}

void TrajectoryFollowController::initialize(const Trajectory& trajectory) {
    this->trajectory_ = trajectory;
    this->state_ = ControllerState::INITIALIZED;
    this->current_index_ = 0;
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

    // x についての制御
    output.x = this->control(
        current_state.pose.x,
        this->trajectory_.poses[this->current_index_ + 1].x,
        this->trajectory_.poses[this->current_index_].x,
        this->trajectory_.dt_ms
    );

    // y についての制御
    output.y = this->control(
        current_state.pose.y,
        this->trajectory_.poses[this->current_index_ + 1].y,
        this->trajectory_.poses[this->current_index_].y,
        this->trajectory_.dt_ms
    );

    // theta についての制御
    output.theta = this->control(
        current_state.pose.theta,
        this->trajectory_.poses[this->current_index_ + 1].theta,
        this->trajectory_.poses[this->current_index_].theta,
        this->trajectory_.dt_ms
    );

    // 終点に到達するまで current_index_ をインクリメント
    if (this->current_index_ < this->trajectory_.poses.size() - 1) {
        this->current_index_++;
    } else {
        state = ControllerState::COMPLETE;
    }   
}

double TrajectoryFollowController::control(double current, double target, double current_target, uint16_t dt_ms) {
    double error = target - current;
    double ideal_speed = (target - current_target) / dt_ms;
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