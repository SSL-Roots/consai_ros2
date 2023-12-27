#include "consai_robot_controller/trajectory_generator.hpp"
#include "consai_robot_controller/trajectory/bangbangtrajectory2d.h"

Trajectory TrajectoryGenerator::generate(
    State2D start_state, State2D end_state, double vmax, double acc, double accuracy
) {
    /**
     * @brief 2次元の軌道を生成する
     * @details 2次元の軌道を生成する。まず、BangBangTrajectory2Dクラスで軌道を生成し、それをTrajectoryクラスに変換する。
    */
    BangBangTrajectory2D trajectory;

    Vector2D s0 = Vector2D(start_state.pose.x, start_state.pose.y);
    Vector2D s1 = Vector2D(end_state.pose.x, end_state.pose.y);
    Vector2D v0 = Vector2D(start_state.velocity.x, start_state.velocity.y);

    trajectory.generate(s0, s1, v0, vmax, acc, accuracy);

    std::vector<Pose2D> poses;
    double total_time = trajectory.get_total_time();
    double dt = 0.1;
    for (double t = 0.0; t < total_time; t += dt) {
        Vector2D position = trajectory.get_position(t);
        double theta = 0; // TODO: ここで角度を計算する
        Pose2D pose = Pose2D(position.x, position.y, theta);
        poses.push_back(pose);
    }

    Trajectory traj = Trajectory(poses, dt * 1000);
    return traj;
}