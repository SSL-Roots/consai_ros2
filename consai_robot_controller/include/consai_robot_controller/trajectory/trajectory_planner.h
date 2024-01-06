#pragma once

#include <vector>
#include <limits>

#include "consai_robot_controller/trajectory/utils.h"
#include "consai_robot_controller/trajectory/field.h"
#include "consai_robot_controller/trajectory/bangbangtrajectory2d.h"
#include "consai_robot_controller/trajectory/obstacle.h"

class StateLatticeTrajectoryPlanner
{
private:
  Field field;
  double v_max;
  double a_max;

  double sample_radius;
  double sample_angle_step;
  double sample_angle_range;
  double look_ahed_distance;

  double robot_radius;
  double dt;

  std::vector < BangBangTrajectory2D > generate_trajectories(Vector2D p0, Vector2D v0, Vector2D pg)
  {
    std::vector < BangBangTrajectory2D > trajectories;

    // 基準となる目的地を生成
    double look_ahead_angle = (pg - p0).angle();
    double distance_to_goal = p0.distance_to(pg);
    double sample_radius;
    if (distance_to_goal < this->sample_radius) {
      sample_radius = distance_to_goal;
    } else {
      sample_radius = this->sample_radius;
    }
    Vector2D temp_goal = p0 + Vector2D(sample_radius, 0).rotate(look_ahead_angle);

    for (double angle = look_ahead_angle; angle < look_ahead_angle + this->sample_angle_range;
      angle += this->sample_angle_step)
    {
      // 目的地を生成
      temp_goal = p0 + Vector2D(sample_radius, 0).rotate(angle);
      // 軌道を生成
      BangBangTrajectory2D traj;
      traj.generate(p0, temp_goal, v0, this->v_max, this->a_max, 0.1);
      trajectories.push_back(traj);
    }

    return trajectories;
  }

  BangBangTrajectory2D get_best_trajectory(
    std::vector < BangBangTrajectory2D > & trajectories,
    Vector2D pg, std::vector < IObstacle * > & obstacles,
    double t = 0)
  {
    // 軌道の範囲内に含まれる障害物のみを評価対象とする
    Vector2D current_position = trajectories[0].get_position(0);
    Circle traj_area(current_position, this->sample_radius);
    std::vector < IObstacle * > filtered_obstacles;
    for (IObstacle * obstacle : obstacles) {
      if (obstacle->does_circle_collide(traj_area)) {
        filtered_obstacles.push_back(obstacle);
      }
    }

    // 評価の各項目を計算
    std::vector < double > distance_list;
    std::vector < std::pair < double, double >> obstacle_eval_list;
    for (BangBangTrajectory2D traj : trajectories) {
      distance_list.push_back(_get_distance_between_tip_of_trajectory_and_goal(traj, pg));
      obstacle_eval_list.push_back(_get_obstacle_evals(traj, filtered_obstacles, t));
    }
    std::vector < double > time_to_collision_list;
    std::vector < double > clearance_list;
    for (auto x : obstacle_eval_list) {
      time_to_collision_list.push_back(x.first);
      clearance_list.push_back(x.second);
    }

    // 評価
    double k0 = 1.0;
    double k1 = 10000.0;
    double k2 = 1.0;
    std::vector < double > evals;
    for (int i = 0; i < distance_list.size(); i++) {
      double distance = distance_list[i];
      double time = time_to_collision_list[i];
      double clearance = clearance_list[i];

      // debug
      double val_k0 = k0 * distance;
      double val_k1 = k1 * std::max((-1.0 / 5.0 * time) + 1.0, 0.0);
      double val_k2 = k2 * std::max((-1.0 / 2.0 * clearance) + 1.0, 0.0);
      // debug

      // double eval = k0 * distance + k1 * std::max((-1 / 5 * time) + 1, 0.0) + k2 * std::max((-1 / 2 * clearance) + 1, 0.0);
      double eval = val_k0 + val_k1 + val_k2;
      evals.push_back(eval);
    }

    // evals と trajectories をペアにしてソート
    std::vector < std::pair < double, BangBangTrajectory2D >> eval_traj_pairs;
    for (int i = 0; i < evals.size(); i++) {
      eval_traj_pairs.push_back(std::make_pair(evals[i], trajectories[i]));
    }
    std::sort(
      eval_traj_pairs.begin(), eval_traj_pairs.end(), [] (const auto & x, const auto & y)
      {return x.first < y.first;});

    // 評価値が最小の軌道を選択
    BangBangTrajectory2D traj = eval_traj_pairs[0].second;

    return traj;
  }

  double _get_distance_between_tip_of_trajectory_and_goal(BangBangTrajectory2D traj, Vector2D goal)
  {
    // 軌道先端のゴールとの距離
    double distance_to_path = traj.get_position(traj.get_total_time()).distance_to(goal);

    return distance_to_path;
  }

  std::pair < double, double > _get_obstacle_evals(
    BangBangTrajectory2D traj,
    std::vector < IObstacle * > &obstacles,
    double t_from_start)
  {
    // 障害物に衝突するまでの時間
    double minimum_clearance = 1e10;        // big constant

    // 軌道の範囲内に含まれる障害物のみを評価対象とする
    Vector2D traj_max_position = traj.get_max_position();
    Vector2D traj_min_position = traj.get_min_position();
    double traj_area_width = traj_max_position.x - traj_min_position.x;
    double traj_area_height = traj_max_position.y - traj_min_position.y;
    NonRotatingRectangle traj_area(
      traj_min_position.x, traj_min_position.y, traj_area_width,
      traj_area_height);
    std::vector < IObstacle * > filtered_obstacles;
    for (IObstacle * obstacle : obstacles) {
      if (obstacle->does_rectangle_collide(traj_area)) {
        filtered_obstacles.push_back(obstacle);
      }
    }

    for (double t = 0; t < traj.get_total_time(); t += this->dt) {
      for (IObstacle * obstacle : filtered_obstacles) {
        Circle robot_circle(traj.get_position(t), this->robot_radius);

        double clearance = obstacle->get_clearance_between_circle(robot_circle);
        minimum_clearance = std::min(clearance, minimum_clearance);

        if (obstacle->does_circle_collide(robot_circle)) {
          return std::make_pair(t, 0);
        }
      }
    }

    return std::make_pair(1e10, minimum_clearance);
  }

public:
  StateLatticeTrajectoryPlanner(Field field, double v_max, double a_max)
    : field(field), v_max(v_max), a_max(a_max),
    sample_radius(2.0), sample_angle_step(M_PI / 12), sample_angle_range(M_PI * 2),
    look_ahed_distance(2.0), robot_radius(0.09), dt(0.1)
  {
    // コンストラクタの本体は、全てのメンバ変数が初期化子で初期化されているため、空です。
  }

  BangBangTrajectory2D planOnce(
    Vector2D p0, Vector2D v0, Vector2D pg,
    std::vector < IObstacle * > obstacles)
  {
    std::vector < BangBangTrajectory2D > trajectories = generate_trajectories(p0, v0, pg);

    BangBangTrajectory2D traj = get_best_trajectory(trajectories, pg, obstacles, 0.0);

    return traj;
  }

  // BangBangTrajectory2D plan(Vector2D p0, Vector2D v0, Vector2D pg, std::vector<IObstacle *> obstacles) {
  //     std::vector<BangBangTrajectory2D> traj_segments;

  //     Vector2D position = p0;
  //     Vector2D velocity = v0;

  //     double t = 0.0;
  //     while (true) {
  //         std::vector<BangBangTrajectory2D> trajectories = generate_trajectories(position, velocity, pg);

  //         BangBangTrajectory2D traj = get_best_trajectory(trajectories, pg, obstacles, t);
  //         BangBangTrajectory2D traj_segement = TrimmedTrajectory(traj, 0.0, 0.1);
  //         traj_segments.push_back(traj_segement);

  //         if (traj_segement.get_position(traj_segement.get_total_time()).distance_to(pg) < 0.001) {
  //             break;
  //         }

  //         position = traj_segement.get_position(traj_segement.get_total_time());
  //         velocity = traj_segement.get_velocity(traj_segement.get_total_time());
  //         t += 0.1;
  //     }

  //     // traj_segmentsをConnectedTrajectoryに変換して返す
  //     BangBangTrajectory2D result_trajectory = traj_segments[0];
  //     for (int i = 1; i < traj_segments.size(); i++) {
  //         result_trajectory = ConnectedTrajectory(result_trajectory, traj_segments[i]);
  //     }

  //     return result_trajectory;
  // }
};
