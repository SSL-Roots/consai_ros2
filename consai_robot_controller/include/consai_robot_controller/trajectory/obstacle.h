#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

// Importing necessary libraries and modules
#include "consai_robot_controller/trajectory/collision.h"
#include "consai_robot_controller/trajectory/utils.h"

using namespace std;

class IObstacle {
public:
    virtual Vector2D get_position() = 0;
    virtual bool does_point_collide(Vector2D point) = 0;
    virtual std::pair<std::pair<bool, Vector2D>, std::pair<bool, Vector2D>> get_intersection_lineseg(LineSegment lineseg) = 0;
    virtual bool does_circle_collide(Circle circle) = 0;
    virtual double get_clearance_between_circle(Circle circle) = 0;
    virtual bool does_rectangle_collide(NonRotatingRectangle rect) = 0;
};

class CircleObstacle : public IObstacle {
private:
    double _x, _y, _r;

public:
    CircleObstacle(double x, double y, double radius) : _x(x), _y(y), _r(radius) {        
    }

    Vector2D get_position() {
        return Vector2D(_x, _y);
    }

    bool does_point_collide(Vector2D point) {
        double distance = get_position().distance_to(point);
        return distance <= _r;
    }

    std::pair<std::pair<bool, Vector2D>, std::pair<bool, Vector2D>>  get_intersection_lineseg(LineSegment lineseg) {
        Circle circle(get_position(), _r);
        return get_intersection_point_circle_lineseg(circle, lineseg);
    }

    bool does_circle_collide(Circle circle) {
        Vector2D myPosition = get_position();
        double distance = myPosition.distance_to(circle.c);
        return distance < _r + circle.r;
    }

    double get_clearance_between_circle(Circle circle) {
        Vector2D myPosition = get_position();
        double distance = myPosition.distance_to(circle.c);
        return std::max(distance - _r, distance - circle.r);
    }

    bool does_rectangle_collide(NonRotatingRectangle rect) {
        Circle myCircle(get_position(), _r);
        return does_collide_non_rotating_rectangle_circle(rect, myCircle);
    }
};

class RectObstacle : public IObstacle {
private:
    double x;
    double y;
    double w;
    double h;

public:
    RectObstacle(double xPos, double yPos, double width, double height) {
        x = xPos;
        y = yPos;
        w = width;
        h = height;
    }

    Vector2D get_position() {
        return Vector2D(x, y);
    }

    bool does_point_collide(Vector2D point) {
        return (x <= point.x && point.x <= x + w) && (y <= point.y && point.y <= y + h);
    }

    std::pair<std::pair<bool, Vector2D>, std::pair<bool, Vector2D>> get_intersection_lineseg(LineSegment lineseg) {
        Rectangle rect(x, y, w, h);
        return get_intersection_point_rectangle_lineseg(rect, lineseg);
    }

    bool does_circle_collide(Circle circle) {
        NonRotatingRectangle rect(x, y, w, h);
        return does_collide_non_rotating_rectangle_circle(rect, circle);
    }

    double get_clearance_between_circle(Circle circle) {
        // To be implemented
        return INFINITY;
    }

    bool does_rectangle_collide(NonRotatingRectangle rect) {
        NonRotatingRectangle myRect(x, y, w, h);
        return does_collide_non_rotating_rectangle_non_rotating_rectangle(myRect, rect);
    }
};
