// Copyright 2023 Roots
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

#include "consai_robot_controller/trajectory/trajectory.hpp"

TrimmedTrajectory::TrimmedTrajectory(ITrajectory * traj, double start_t, double end_t)
: trajectory(traj), start_time(start_t), end_time(end_t) {}

Vector2D TrimmedTrajectory::get_position(double t)
{
  return trajectory->get_position(start_time + t);
}

Vector2D TrimmedTrajectory::get_position_mm(double t)
{
  return trajectory->get_position_mm(start_time + t);
}

Vector2D TrimmedTrajectory::get_velocity(double t)
{
  return trajectory->get_velocity(start_time + t);
}

Vector2D TrimmedTrajectory::get_acceleration(double t)
{
  return trajectory->get_acceleration(start_time + t);
}

double TrimmedTrajectory::get_total_time()
{
  return end_time - start_time;
}

PosVelAcc TrimmedTrajectory::get_values_at_time(double tt)
{
  return trajectory->get_values_at_time(start_time + tt);
}

std::vector<double> TrimmedTrajectory::get_time_sections()
{
  return trajectory->get_time_sections();
}

ConnectedTrajectory::ConnectedTrajectory(ITrajectory * traj1, ITrajectory * traj2)
: traj1(traj1), traj2(traj2) {}

Vector2D ConnectedTrajectory::get_position(double t)
{
  if (t < traj1->get_total_time()) {
    return traj1->get_position(t);
  } else {
    return traj2->get_position(t - traj1->get_total_time());
  }
}

Vector2D ConnectedTrajectory::get_position_mm(double t)
{
  if (t < traj1->get_total_time()) {
    return traj1->get_position_mm(t);
  } else {
    return traj2->get_position_mm(t - traj1->get_total_time());
  }
}

Vector2D ConnectedTrajectory::get_velocity(double t)
{
  if (t < traj1->get_total_time()) {
    return traj1->get_velocity(t);
  } else {
    return traj2->get_velocity(t - traj1->get_total_time());
  }
}

Vector2D ConnectedTrajectory::get_acceleration(double t)
{
  if (t < traj1->get_total_time()) {
    return traj1->get_acceleration(t);
  } else {
    return traj2->get_acceleration(t - traj1->get_total_time());
  }
}

double ConnectedTrajectory::get_total_time()
{
  return traj1->get_total_time() + traj2->get_total_time();
}

PosVelAcc ConnectedTrajectory::get_values_at_time(double t)
{
  if (t < traj1->get_total_time()) {
    return traj1->get_values_at_time(t);
  } else {
    return traj2->get_values_at_time(t - traj1->get_total_time());
  }
}

std::vector<double> ConnectedTrajectory::get_time_sections()
{
  std::vector<double> sections1 = traj1->get_time_sections();
  std::vector<double> sections2 = traj2->get_time_sections();

  std::vector<double> sections;
  sections.reserve(sections1.size() + sections2.size());
  sections.insert(sections.end(), sections1.begin(), sections1.end());
  sections.insert(sections.end(), sections2.begin(), sections2.end());

  return sections;
}
