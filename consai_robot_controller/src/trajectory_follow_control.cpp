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