#include <cmath>
#include <vector>

#include "consai_robot_controller/trajectory/trajectory.h"
#include "consai_robot_controller/trajectory/bangbangtrajectory3d.h"

BangBangTrajectory3D::BangBangTrajectory3D() {
    this->linear_ = BangBangTrajectory2D();
    this->angular_ = BangBangTrajectory1D();
}

BangBangTrajectory3D::~BangBangTrajectory3D() {
}

Pose2D BangBangTrajectory3D::get_pose(double t) {
    Vector2D position_linear = this->linear_.get_position(t);
    return Pose2D(position_linear.x, position_linear.y, this->angular_.get_position(t));
}

Velocity2D BangBangTrajectory3D::get_velocity(double t) {
    Vector2D velocity_linear = this->linear_.get_velocity(t);
    return Velocity2D(velocity_linear.x, velocity_linear.y, this->angular_.get_velocity(t));
}

double BangBangTrajectory3D::get_total_time() {
    return std::max(this->linear_.get_total_time(), this->angular_.get_total_time());
}


void BangBangTrajectory3D::generate(
    Pose2D s0, Pose2D s1, Velocity2D v0, double vmax_linear, double vmax_angular, double acc_linear, double acc_angular, double accuracy
) {
    // 平行移動の軌道を生成
    this->linear_.generate(
        Vector2D(s0.x, s0.y), Vector2D(s1.x, s1.y), Vector2D(v0.x, v0.y), vmax_linear, acc_linear, accuracy
    );

    // 回転の軌道を生成
    // 正転と逆転の2つの軌道を生成して、短時間で到達する軌道を採用する
    BangBangTrajectory1D trajectory0, trajectory1;

    // 現在角度に対して正方向と逆方向の2つの目標角度を生成する
    double target_theta0, target_theta1;
    if (s1.theta > s0.theta) {
        target_theta0 = s1.theta;
        target_theta1 = s1.theta - 2.0 * M_PI;
    } else {
        target_theta0 = s1.theta + 2.0 * M_PI;
        target_theta1 = s1.theta;
    }

    trajectory0.generate(s0.theta, target_theta0, v0.theta, vmax_angular, acc_angular);
    trajectory1.generate(s0.theta, target_theta1, v0.theta, vmax_angular, acc_angular);

    // 2つの軌道の到達時間を比較して、短い方を採用する
    if (trajectory0.get_total_time() < trajectory1.get_total_time()) {
        this->angular_ = trajectory0;
    } else {
        this->angular_ = trajectory1;
    }
}