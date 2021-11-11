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

#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include "consai_vision_tracker/robot_tracker.hpp"
#include "robocup_ssl_msgs/msg/vector2.hpp"

namespace consai_vision_tracker
{

using ColumnVector = BFL::ColumnVector;
using Matrix = BFL::Matrix;
using MatrixWrapper = BFL::Matrix_Wrapper;
using SymmetricMatrix = BFL::SymmetricMatrix;
using Vector2 = robocup_ssl_msgs::msg::Vector2;

// visibilityの操作量
// 追跡対象の情報が来ないとき、この操作量だけvisibilityを小さくする
// 操作量 = 1.0 / (trackerの更新周波数 * 消失判定までに掛ける時間)
// 例：trackerが100Hzで更新されるとき、操作量が0.002であれば、
// 追跡対象が消失してから5秒かけてvisibilityが1.0から0.0になる
static const double VISIBILITY_CONTROL_VALUE = 0.005;

RobotTracker::RobotTracker(const int team_color, const int id, const double dt)
{
  prev_tracked_robot_.robot_id.team_color = team_color;
  prev_tracked_robot_.robot_id.id = id;
  // visibilityはoptionalなので、ここでデフォルト値を設定しておく
  prev_tracked_robot_.visibility.push_back(1.0);
  // velocityはoptionalなので、ここでデフォルト値を設定しておく
  prev_tracked_robot_.vel.push_back(Vector2());
  prev_tracked_robot_.vel_angular.push_back(0.0);

  // システムモデル
  // pos(t+1) = pos(t) + vel(t)*dt + undefined_input(t) + noise
  // pos = (x, y, theta)
  Matrix A(6, 6);
  A(1, 1) = 1.0; A(1, 2) = 0.0; A(1, 3) = 0.0; A(1, 4) = dt;  A(1, 5) = 0.0; A(1, 6) = 0.0;
  A(2, 1) = 0.0; A(2, 2) = 1.0; A(2, 3) = 0.0; A(2, 4) = 0.0; A(2, 5) = dt;  A(2, 6) = 0.0;
  A(3, 1) = 0.0; A(3, 2) = 0.0; A(3, 3) = 1.0; A(3, 4) = 0.0; A(3, 5) = 0.0; A(3, 6) = dt;
  A(4, 1) = 0.0; A(4, 2) = 0.0; A(4, 3) = 0.0; A(4, 4) = 1.0; A(4, 5) = 0.0; A(4, 6) = 0.0;
  A(5, 1) = 0.0; A(5, 2) = 0.0; A(5, 3) = 0.0; A(5, 4) = 0.0; A(5, 5) = 1.0; A(5, 6) = 0.0;
  A(6, 1) = 0.0; A(6, 2) = 0.0; A(6, 3) = 0.0; A(6, 4) = 0.0; A(6, 5) = 0.0; A(6, 6) = 1.0;

  // 入力は未実装
  Matrix B(6, 3);
  B(1, 1) = 0.0; B(1, 2) = 0.0; B(1, 3) = 0.0;
  B(2, 1) = 0.0; B(2, 2) = 0.0; B(2, 3) = 0.0;
  B(3, 1) = 0.0; B(3, 2) = 0.0; B(3, 3) = 0.0;
  B(4, 1) = 0.0; B(4, 2) = 0.0; B(4, 3) = 0.0;
  B(5, 1) = 0.0; B(5, 2) = 0.0; B(5, 3) = 0.0;
  B(6, 1) = 0.0; B(6, 2) = 0.0; B(6, 3) = 0.0;

  std::vector<Matrix> AB(2);
  AB[0] = A;
  AB[1] = B;

  // システムノイズの平均値
  ColumnVector sys_noise_mu(6);
  sys_noise_mu = 0.0;

  // 位置、速度の変化をのシステムノイズで表現する（つまりめちゃくちゃノイズがでかい）
  // 0 m/s から、いきなり1.0 m/sに変化しうる、ということ
  // 例：1.0[m/s] / 0.001[s] = 100 [m/ss]
  const double MAX_LINEAR_ACC_MPS = 0.1 * 1.0 / dt;
  // 例：1.0[rad/s] / 0.001[s] = 100 [rad/ss]
  const double MAX_ANGULAR_ACC_RADPS = 0.05 * M_PI / dt;
  const double MAX_LINEAR_ACCEL_IN_DT = MAX_LINEAR_ACC_MPS * dt;  // [m/s]
  const double MAX_ANGULAR_ACCEL_IN_DT = MAX_ANGULAR_ACC_RADPS * dt;  // [rad/s]
  const double MAX_LINEAR_MOVEMENT_IN_DT = MAX_LINEAR_ACC_MPS / 2 * std::pow(dt, 2);  // [m]
  const double MAX_ANGULAR_MOVEMENT_IN_DT = MAX_ANGULAR_ACC_RADPS / 2 * std::pow(dt, 2);  // [rad]

  // システムノイズの分散
  SymmetricMatrix sys_noise_cov(6);
  sys_noise_cov = 0.0;
  sys_noise_cov(1, 1) = std::pow(MAX_LINEAR_MOVEMENT_IN_DT, 2);
  sys_noise_cov(2, 2) = std::pow(MAX_LINEAR_MOVEMENT_IN_DT, 2);
  sys_noise_cov(3, 3) = std::pow(MAX_ANGULAR_MOVEMENT_IN_DT, 2);
  sys_noise_cov(4, 4) = std::pow(MAX_LINEAR_ACCEL_IN_DT, 2);
  sys_noise_cov(5, 5) = std::pow(MAX_LINEAR_ACCEL_IN_DT, 2);
  sys_noise_cov(6, 6) = std::pow(MAX_ANGULAR_ACCEL_IN_DT, 2);

  Gaussian system_uncertainty(sys_noise_mu, sys_noise_cov);
  sys_pdf_ = std::make_shared<ConditionalGaussian>(AB, system_uncertainty);
  sys_model_ = std::make_shared<SystemModelGaussianUncertainty>(sys_pdf_.get());

  // 観測モデル
  // ~pos(t) = pos(t) + noise
  // pos = (x, y, theta)
  Matrix H(3, 6);
  H = 0.0;
  H(1, 1) = 1.0;
  H(2, 2) = 1.0;
  H(3, 3) = 1.0;

  // 観測ノイズの平均値
  ColumnVector meas_noise_mu(3);
  meas_noise_mu = 0.0;

  // 観測ノイズの分散
  SymmetricMatrix meas_noise_cov(3);
  meas_noise_cov(1, 1) = std::pow(0.02, 2);
  meas_noise_cov(2, 2) = std::pow(0.02, 2);
  meas_noise_cov(3, 3) = std::pow(0.02 * M_PI, 2);
  Gaussian measurement_uncertainty(meas_noise_mu, meas_noise_cov);

  meas_pdf_ = std::make_shared<ConditionalGaussian>(H, measurement_uncertainty);
  meas_model_ =
    std::make_shared<MeasurementModelGaussianUncertainty>(meas_pdf_.get());

  // 事前分布
  ColumnVector prior_mu(6);
  prior_mu = 0.0;

  SymmetricMatrix prior_cov(6);
  prior_cov = 0.0;
  prior_cov(1, 1) = 100.0;
  prior_cov(2, 2) = 100.0;
  prior_cov(3, 3) = 100.0;
  prior_cov(4, 4) = 100.0;
  prior_cov(5, 5) = 100.0;
  prior_cov(6, 6) = 100.0;

  prior_ = std::make_shared<Gaussian>(prior_mu, prior_cov);

  // カルマンフィルタの生成
  filter_ = std::make_shared<ExtendedKalmanFilter>(prior_.get());
}

void RobotTracker::push_back_observation(const DetectionRobot & robot)
{
  // 角度データを含まない場合は、そのデータを参照しない
  if (robot.orientation.size() == 0) {
    return;
  }

  TrackedRobot observation;
  observation.pos.x = robot.x * 0.001;  // mm to meters
  observation.pos.y = robot.y * 0.001;  // mm to meters
  observation.orientation = robot.orientation[0];
  robot_observations_.push_back(observation);
}

TrackedRobot RobotTracker::update()
{
  // 観測値から外れ値を取り除く
  for (auto it = robot_observations_.begin(); it != robot_observations_.end(); ) {
    if (is_outlier(*it)) {
      it = robot_observations_.erase(it);
    } else {
      ++it;
    }
  }

  auto size = robot_observations_.size();
  if (size == 0) {
    // 観測値が無い場合の処理
    // visibilityを下げる
    prev_tracked_robot_.visibility[0] -= VISIBILITY_CONTROL_VALUE;
    if (prev_tracked_robot_.visibility[0] <= 0) {
      // visibilityが0になったらカルマンフィルタの演算を実行しない
      prev_tracked_robot_.visibility[0] = 0.0;
      reset_prior();
      return prev_tracked_robot_;
    }

  } else {
    // 観測値があればvisibilityをn倍のレートで上げる
    prev_tracked_robot_.visibility[0] += VISIBILITY_CONTROL_VALUE * 5.0;
    if (prev_tracked_robot_.visibility[0] > 1.0) {
      prev_tracked_robot_.visibility[0] = 1.0;
    }

    // 観測値が複数ある場合は、その平均値をもとめる
    // この処理で観測値を全て削除する
    ColumnVector mean_observation(3);
    mean_observation(1) = 0.0;
    mean_observation(2) = 0.0;
    double sum_x = 0.0;
    double sum_y = 0.0;

    for (auto it = robot_observations_.begin(); it != robot_observations_.end(); ) {
      mean_observation(1) += it->pos.x;
      mean_observation(2) += it->pos.y;
      // 角度は-pi ~ piの範囲なので、2次元ベクトルに変換してから平均値を求める
      sum_x += std::cos(it->orientation);
      sum_y += std::sin(it->orientation);
      it = robot_observations_.erase(it);
    }
    mean_observation(1) /= size;
    mean_observation(2) /= size;
    sum_x /= size;
    sum_y /= size;
    mean_observation(3) = std::fmod(std::atan2(sum_y, sum_x), M_PI);

    // 観測値と前回の予測値がpi, -pi付近にあるとき、
    // ２つの角度の差分が大きくならないように、観測値の符号と値を調節する
    mean_observation(3) = normalize_orientation(
      filter_->PostGet()->ExpectedValueGet()(3), mean_observation(3));

    filter_->Update(meas_model_.get(), mean_observation);
    correct_orientation_overflow_of_prior();
  }
  // 事後分布から予測値を取得
  auto expected_value = filter_->PostGet()->ExpectedValueGet();
  prev_tracked_robot_.pos.x = expected_value(1);
  prev_tracked_robot_.pos.y = expected_value(2);
  prev_tracked_robot_.orientation = expected_value(3);
  prev_tracked_robot_.vel[0].x = expected_value(4);
  prev_tracked_robot_.vel[0].y = expected_value(5);
  prev_tracked_robot_.vel_angular[0] = expected_value(6);

  // 次の状態を予測する
  // 例えば、ロボットの加速度が入力値になる
  // 入力値は未実装なので0とする
  ColumnVector input(3);
  input(1) = 0;
  input(2) = 0;
  input(3) = 0;
  filter_->Update(sys_model_.get(), input);
  correct_orientation_overflow_of_prior();

  return prev_tracked_robot_;
}

void RobotTracker::reset_prior()
{
  // 事前分布を初期化する
  ColumnVector prior_mu(6);
  prior_mu = 0.0;

  SymmetricMatrix prior_cov(6);
  prior_cov = 0.0;
  prior_cov(1, 1) = 100.0;
  prior_cov(2, 2) = 100.0;
  prior_cov(3, 3) = 100.0;
  prior_cov(4, 4) = 100.0;
  prior_cov(5, 5) = 100.0;
  prior_cov(6, 6) = 100.0;

  prior_->ExpectedValueSet(prior_mu);
  prior_->CovarianceSet(prior_cov);
  filter_->Reset(prior_.get());
}

bool RobotTracker::is_outlier(const TrackedRobot & observation) const
{
  // 観測が外れ値かどうか判定する
  // Reference: https://myenigma.hatenablog.com/entry/20140825/1408975706
  const double THRESHOLD = 5.99;  // 自由度2、棄却率5%のしきい値

  auto expected_value = filter_->PostGet()->ExpectedValueGet();
  auto covariance = filter_->PostGet()->CovarianceGet();

  // マハラノビス距離を求める
  double diff_x = observation.pos.x - expected_value(1);
  double diff_y = observation.pos.y - expected_value(2);
  double covariance_x = covariance(1, 1);
  double covariance_y = covariance(2, 2);

  // 0 除算を避ける
  if (std::fabs(covariance_x) < 1E-15 || std::fabs(covariance_y) < 1E-15) {
    return false;
  }

  double mahalanobis = std::sqrt(
    std::pow(diff_x, 2) / covariance_x + std::pow(
      diff_y,
      2) / covariance_y);
  if (mahalanobis > THRESHOLD) {
    return true;
  }

  return false;
}

void RobotTracker::correct_orientation_overflow_of_prior()
{
  // 事後分布の角度を取得し、-pi ~ piの範囲に収め、事前分布にセットする
  auto expected_value = filter_->PostGet()->ExpectedValueGet();
  auto covariance = filter_->PostGet()->CovarianceGet();

  if (expected_value(3) < -M_PI || expected_value(3) > M_PI) {
    expected_value(3) = normalize_orientation(expected_value(3));

    prior_->ExpectedValueSet(expected_value);
    prior_->CovarianceSet(covariance);

    filter_->Reset(prior_.get());
  }
}

double RobotTracker::normalize_orientation(double orientation) const
{
  // 角度を-pi ~ piの範囲に収める
  while (orientation >= M_PI) {orientation -= 2.0 * M_PI;}
  while (orientation <= -M_PI) {orientation += 2.0 * M_PI;}
  return orientation;
}

double RobotTracker::normalize_orientation(const double from, const double to) const
{
  // fromからtoへ連続に角度が変化するようにtoの符号と大きさを変更する
  // from(150 deg) -> to(-150 deg) => from(150 deg) -> to(210 deg)
  return from + normalize_orientation(to - from);
}

}  // namespace consai_vision_tracker
