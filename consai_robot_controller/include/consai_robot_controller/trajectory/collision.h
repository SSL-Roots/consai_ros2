#pragma once

#include <iostream>
#include <cmath>
#include "trajectory.h"
#include "utils.h"

bool does_collide_lineseg_lineseg(LineSegment seg1, LineSegment seg2) {
    double eps = 1e-6;

    double a_x = seg1.s.x;
    double a_y = seg1.s.y;
    double b_x = seg1.e.x;
    double b_y = seg1.e.y;
    double c_x = seg2.s.x;
    double c_y = seg2.s.y;
    double d_x = seg2.e.x;
    double d_y = seg2.e.y;

    double s = (a_x - b_x) * (c_y - a_y) - (a_y - b_y) * (c_x - a_x);
    double t = (a_x - b_x) * (d_y - a_y) - (a_y - b_y) * (d_x - a_x);
    if (s * t > -eps) {
        return false;
    }

    s = (c_x - d_x) * (a_y - c_y) - (c_y - d_y) * (a_x - c_x);
    t = (c_x - d_x) * (b_y - c_y) - (c_y - d_y) * (b_x - c_x);
    if (s * t > -eps) {
        return false;
    }
    
    return true;
}

std::pair<bool, Vector2D> get_intersection_point_lineseg_lineseg(LineSegment seg1, LineSegment seg2) {
    double eps = 1e-6;

    double a_x = seg1.s.x;
    double a_y = seg1.s.y;
    double b_x = seg1.e.x;
    double b_y = seg1.e.y;
    double c_x = seg2.s.x;
    double c_y = seg2.s.y;
    double d_x = seg2.e.x;
    double d_y = seg2.e.y;

    double s = (a_x - b_x) * (c_y - a_y) - (a_y - b_y) * (c_x - a_x);
    double t = (a_x - b_x) * (d_y - a_y) - (a_y - b_y) * (d_x - a_x);
    if (s * t > -eps) {
        return { false, Vector2D() };
    }

    s = (c_x - d_x) * (a_y - c_y) - (c_y - d_y) * (a_x - c_x);
    t = (c_x - d_x) * (b_y - c_y) - (c_y - d_y) * (b_x - c_x);
    if (s * t > -eps) {
        return { false, Vector2D() };
    }
    
    double denominator = (a_x - b_x) * (c_y - d_y) - (a_y - b_y) * (c_x - d_x);
    if (fabs(denominator) < eps) {
        return { false, Vector2D()};
    }
    
    double a = ((c_x - d_x) * (c_y - a_y) - (c_y - d_y) * (c_x - a_x)) / denominator;
    double x = a_x + a * (b_x - a_x);
    double y = a_y + a * (b_y - a_y);
    return {true, Vector2D(x, y)};
}

bool does_collide_rectangle_lineseg(Rectangle rect, LineSegment seg) {
    // Check if a rectangle and a line segment collide

    for (auto s : rect.sides) {
        if (does_collide_lineseg_lineseg(s, seg)) {
            return true;
        }
    }

    if (rect.does_point_include(seg.s) && rect.does_point_include(seg.e)) {
        return true;
    }

    for (auto s : rect.diagonals) {
        if (does_collide_lineseg_lineseg(s, seg)) {
            return true;
        }
    }

    return false;
}

std::pair<std::pair<bool, Vector2D>, std::pair<bool, Vector2D>> get_intersection_point_rectangle_lineseg(Rectangle rect, LineSegment seg) {
    // Get the intersection points of a rectangle and a line segment

    std::vector<Vector2D> intersection_points;

    for (auto s : rect.sides) {
        auto intersection_point = get_intersection_point_lineseg_lineseg(s, seg);
        if (intersection_point.first) {
            intersection_points.push_back(intersection_point.second);
        }
    }

    if (intersection_points.empty()) {
        return { std::make_pair(false, Vector2D()), std::make_pair(false, Vector2D()) };
    }

    if (intersection_points.size() == 1) {
        return { std::make_pair(true, intersection_points[0]), std::make_pair(false, Vector2D()) };
    }

    return { std::make_pair(true, intersection_points[0]), std::make_pair(true, intersection_points[1]) };
}

bool does_collide_circle_lineseg(Circle circle, LineSegment seg) {
    // Check if a circle and a line segment collide

    if (seg.distance_to_point(circle.c) < circle.r) {
        return true;
    }

    return false;
}

std::pair<std::pair<bool, Vector2D>, std::pair<bool, Vector2D>> get_intersection_point_circle_lineseg(Circle circle, LineSegment seg) {
    // Get the intersection points of a circle and a line segment

    Vector2D l = seg.e - seg.s;
    Vector2D c = circle.c - seg.s;

    Vector2D l_unit = l / sqrt(l.x * l.x + l.y * l.y);
    double dot = l_unit.x * c.x + l_unit.y * c.y;
    Vector2D proj = l_unit * dot;

    double h = abs(l_unit.x * c.y - l_unit.y * c.x);

    if (h > circle.r) {
        return { { false, Vector2D()}, { false, Vector2D()} };
    }

    double d = sqrt(circle.r * circle.r - h * h);

    Vector2D p1_temp = proj - l_unit * d;
    Vector2D p2_temp = proj + l_unit * d;

    std::pair<bool, Vector2D> p1, p2;

    if (l.x * p1_temp.x + l.y * p1_temp.y < 0 || sqrt(p1_temp.x * p1_temp.x + p1_temp.y * p1_temp.y) > sqrt(l.x * l.x + l.y * l.y)) {
        p1 = { false, Vector2D() };
    }
    else {
        p1 = {true, p1_temp + seg.s};
    }

    if (l.x * p2_temp.x + l.y * p2_temp.y < 0 || sqrt(p2_temp.x * p2_temp.x + p2_temp.y * p2_temp.y) > sqrt(l.x * l.x + l.y * l.y)) {
        p2 =  { false, Vector2D() };
    }
    else {
        p2 = { true, p2_temp + seg.s};
    }

    return {p1, p2};    
}

bool does_collide_circle_rectangle(const Circle& circle, Rectangle& rect) {
    // Checking if any of the sides of the rectangle intersect with the circle
    for (const auto& side : rect.sides) {
        if (does_collide_circle_lineseg(circle, side)) {
            return true;
        }
    }

    // Checking if the center of the circle is inside the rectangle
    if (rect.does_point_include(circle.c)) {
        return true;
    }

    return false;
}

bool does_collide_rectangle_rectangle(Rectangle& rect1, Rectangle& rect2) {
    // Checking if any of the sides of rect1 intersect with rect2
    for (const auto& side : rect1.sides) {
        if (does_collide_rectangle_lineseg(rect2, side)) {
            return true;
        }
    }

    // Checking if any of the sides of rect2 intersect with rect1
    for (const auto& side : rect2.sides) {
        if (does_collide_rectangle_lineseg(rect1, side)) {
            return true;
        }
    }

    // Checking if any vertex of rect1 is inside rect2
    for (const auto& vertex : rect1.get_vertices()) {
        if (rect2.does_point_include(vertex)) {
            return true;
        }
    }

    // Checking if any vertex of rect2 is inside rect1
    for (const auto& vertex : rect2.get_vertices()) {
        if (rect1.does_point_include(vertex)) {
            return true;
        }
    }

    return false;
}

bool does_collide_non_rotating_rectangle_circle(const NonRotatingRectangle& rect, const Circle& circle) {
    // Checking in the x-direction
    if (circle.c.x + circle.r < rect.x) {
        return false;
    }
    if (circle.c.x - circle.r > rect.x + rect.w) {
        return false;
    }

    // Checking in the y-direction
    if (circle.c.y + circle.r < rect.y) {
        return false;
    }
    if (circle.c.y - circle.r > rect.y + rect.h) {
        return false;
    }

    // Calculating the distance between the corners of the rectangle and the circle
    // If this distance is less than the r of the circle, then they collide
    double dx = std::max(std::max(rect.x - circle.c.x, 0.0), circle.c.x - (rect.x + rect.w));
    double dy = std::max(std::max(rect.y - circle.c.y, 0.0), circle.c.y - (rect.y + rect.h));

    return std::pow(dx, 2) + std::pow(dy, 2) < std::pow(circle.r, 2);
}

bool does_collide_non_rotating_rectangle_non_rotating_rectangle(const NonRotatingRectangle& rect1, const NonRotatingRectangle& rect2) {
    // Checking in the x-direction
    if (rect1.left_top.x + rect1.w < rect2.left_top.x) {
        return false;
    }
    if (rect1.left_top.x > rect2.left_top.x + rect2.w) {
        return false;
    }

    // Checking in the y-direction
    if (rect1.left_top.y + rect1.h < rect2.left_top.y) {
        return false;
    }
    if (rect1.left_top.y > rect2.left_top.y + rect2.h) {
        return false;
    }

    return true;
}