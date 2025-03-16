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

// utils.h
#pragma once

#include <vector>

#include "consai_msgs/msg/state2_d.hpp"


class PosVelAcc
{
public:
  PosVelAcc(double pos, double vel, double acc);

  double pos;
  double vel;
  double acc;
};

class Vector2D
{
public:
  Vector2D()
  {
    x = 0;
    y = 0;
  }
  Vector2D(double x, double y);

  double distance_to(Vector2D other);
  double norm();
  double angle();
  Vector2D normalize();
  Vector2D rotate(double angle);
  double dot(Vector2D other);
  double cross(Vector2D other);
  Vector2D operator+(Vector2D other);
  Vector2D operator-(Vector2D other);
  Vector2D operator*(double scalar);
  Vector2D operator/(double scalar);

  double x;
  double y;
};

class Pose2D
{
public:
  double x;
  double y;
  double theta;

  Pose2D();
  Pose2D(double x, double y, double theta);
  Pose2D(const consai_msgs::msg::State2D &state2d);

  consai_msgs::msg::State2D toState2DMsg() const;

  bool operator==(const Pose2D &other) const {
    return x == other.x && y == other.y && theta == other.theta;
  }
  bool operator!=(const Pose2D &other) const {
    return !(*this == other);
  }
};

class Velocity2D
{
public:
  double x;
  double y;
  double theta;

  Velocity2D();
  Velocity2D(double x, double y, double theta);

  consai_msgs::msg::State2D toState2DMsg() const;
};

class State2D
{
public:
  Pose2D pose;
  Velocity2D velocity;

  State2D();
  State2D(Pose2D pose, Velocity2D velocity);
};


class LineSegment
{
public:
  LineSegment(Vector2D start, Vector2D end);

  Vector2D s;
  Vector2D e;

  double length();
  double distance_to_point(Vector2D point);
  Vector2D closest_point(Vector2D point);
};

class Circle
{
public:
  Circle(Vector2D center, double radius);

  Vector2D c;
  double r;
};

class IPolygon
{
public:
  virtual bool does_point_include(Vector2D p) = 0;
  virtual std::vector<Vector2D> get_vertices() = 0;
};

class Rectangle : public IPolygon
{
public:
  double x, y, w, h;
  Vector2D left_bottom;
  Vector2D left_top;
  Vector2D right_top;
  Vector2D right_bottom;
  std::vector<LineSegment> sides;
  std::vector<LineSegment> diagonals;

  Rectangle(double x, double y, double w, double h)
  : x(x), y(y), w(w), h(h),
    left_bottom(Vector2D(x, y)),
    left_top(Vector2D(x, y + h)),
    right_top(Vector2D(x + w, y + h)),
    right_bottom(Vector2D(x + w, y)),
    sides(
      {LineSegment(left_bottom, left_top),
        LineSegment(left_top, right_top),
        LineSegment(right_top, right_bottom),
        LineSegment(right_bottom, left_bottom)}),
    diagonals(
      {LineSegment(left_bottom, right_top),
        LineSegment(left_top, right_bottom)})
  {
  }

  bool does_point_include(Vector2D p) override
  {
    if (p.x <= x || p.x >= x + w) {
      return false;
    }
    if (p.y <= y || p.y >= y + h) {
      return false;
    }
    return true;
  }

  std::vector<Vector2D> get_vertices() override
  {
    return {left_bottom, left_top, right_top, right_bottom};
  }

  // void plot(Axes& ax, std::string color = "k", bool fill = false) {
  //     std::vector<Vector2D> vertices = get_vertices();
  //     vertices.push_back(vertices[0]);
  //     std::vector<double> xs;
  //     std::vector<double> ys;
  //     for (const Vector2D& v : vertices) {
  //         xs.push_back(v.x);
  //         ys.push_back(v.y);
  //     }
  //     if (fill) {
  //         ax.fill(xs, ys, color);
  //     } else {
  //         ax.plot(xs, ys, color);
  //     }
  // }
};

class NonRotatingRectangle : public IPolygon
{
public:
  double x, y, w, h;
  Vector2D left_bottom;
  Vector2D left_top;
  Vector2D right_top;
  Vector2D right_bottom;
  std::vector<LineSegment> sides;
  std::vector<LineSegment> diagonals;

  NonRotatingRectangle(double x, double y, double w, double h)
  : x(x), y(y), w(w), h(h),
    left_bottom(Vector2D(x, y)),
    left_top(Vector2D(x, y + h)),
    right_top(Vector2D(x + w, y + h)),
    right_bottom(Vector2D(x + w, y)),
    sides(
      {LineSegment(left_bottom, left_top),
        LineSegment(left_top, right_top),
        LineSegment(right_top, right_bottom),
        LineSegment(right_bottom, left_bottom)}),
    diagonals(
      {LineSegment(left_bottom, right_top),
        LineSegment(left_top, right_bottom)})
  {
  }

  bool does_point_include(Vector2D p) override
  {
    if (p.x <= x || p.x >= x + w) {
      return false;
    }
    if (p.y <= y || p.y >= y + h) {
      return false;
    }
    return true;
  }

  std::vector<Vector2D> get_vertices() override
  {
    return {left_bottom, left_top, right_top, right_bottom};
  }

  // void plot(Axes& ax, std::string color = "k", bool fill = false) {
  //     std::vector<Vector2D> vertices = get_vertices();
  //     vertices.push_back(vertices[0]);
  //     std::vector<double> xs;
  //     std::vector<double> ys;
  //     for (const Vector2D& v : vertices) {
  //         xs.push_back(v.x);
  //         ys.push_back(v.y);
  //     }
  //     if (fill) {
  //         ax.fill(xs, ys, color);
  //     } else {
  //         ax.plot(xs, ys, color);
  //     }
  // }
};
