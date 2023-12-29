#include "consai_robot_controller/trajectory/utils.h"

#include <iostream>
#include <cmath>

using namespace std;

PosVelAcc::PosVelAcc(double pos, double vel, double acc) : pos(pos), vel(vel), acc(acc) {}

Vector2D::Vector2D(double x, double y) : x(x), y(y) {}

double Vector2D::distance_to(Vector2D other) {
    return sqrt(pow(x - other.x, 2) + pow(y - other.y, 2));
}

double Vector2D::norm() {
    return sqrt(pow(x, 2) + pow(y, 2));
}

double Vector2D::angle() {
    return atan2(y, x);
}

Vector2D Vector2D::normalize() {
    double n = norm();
    return Vector2D(x / n, y / n);
}

Vector2D Vector2D::rotate(double angle) {
    double c = cos(angle);
    double s = sin(angle);
    return Vector2D(x * c - y * s, x * s + y * c);
}

double Vector2D::dot(Vector2D other) {
    return x * other.x + y * other.y;
}

double Vector2D::cross(Vector2D other) {
    return x * other.y - y * other.x;
}

Vector2D Vector2D::operator+(Vector2D other) {
    return Vector2D(x + other.x, y + other.y);
}

Vector2D Vector2D::operator-(Vector2D other) {
    return Vector2D(x - other.x, y - other.y);
}

Vector2D Vector2D::operator*(double scalar) {
    return Vector2D(x * scalar, y * scalar);
}

Vector2D Vector2D::operator/(double scalar) {
    return Vector2D(x / scalar, y / scalar);
}


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


LineSegment::LineSegment(Vector2D start, Vector2D end) : s(start), e(end) {}

double LineSegment::length() {
    return s.distance_to(e);
}


double LineSegment::distance_to_point(Vector2D point) {
    Vector2D v = e - s;
    Vector2D sp = point - s;
    Vector2D ep = point - e;

    if (v.dot(sp) < 0) {
        return sp.norm();
    } else if (v.dot(ep) > 0) {
        return ep.norm();
    } else {
        return abs(v.cross(sp)) / v.norm();
    }       
}

Vector2D LineSegment::closest_point(Vector2D point) {
    Vector2D v = e - s;
    Vector2D sp = point - s;
    Vector2D ep = point - e;

    if (v.dot(sp) < 0) {
        return s;
    } else if (v.dot(ep) > 0) {
        return e;
    } else {
        return s + v * (v.dot(sp) / v.dot(v));
    }       
}


Circle::Circle(Vector2D center, double radius) : c(center), r(radius) {}



