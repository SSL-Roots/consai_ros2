// trajectory.h
#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>
#include "consai_robot_controller/trajectory/utils.h"

class ITrajectory {
public:
    virtual Vector2D get_position(double t) = 0;
    virtual Vector2D get_position_mm(double t) = 0;
    virtual Vector2D get_velocity(double t) = 0;
    virtual Vector2D get_acceleration(double t) = 0;
    virtual double get_total_time() = 0;
    virtual PosVelAcc get_values_at_time(double tt) = 0;
    virtual std::vector<double> get_time_sections() = 0;
};

class TrimmedTrajectory : public ITrajectory {
private:
    ITrajectory* trajectory;
    double start_time;
    double end_time;

public:
    TrimmedTrajectory(ITrajectory* traj, double start_t, double end_t);
    Vector2D get_position(double t);
    Vector2D get_position_mm(double t);
    Vector2D get_velocity(double t);
    Vector2D get_acceleration(double t);
    double get_total_time();
    PosVelAcc get_values_at_time(double tt);
    std::vector<double> get_time_sections();
};

class ConnectedTrajectory : public ITrajectory {
private:
    ITrajectory* traj1;
    ITrajectory* traj2;

public:
    ConnectedTrajectory(ITrajectory* t1, ITrajectory* t2);
    Vector2D get_position(double t);
    Vector2D get_position_mm(double t);
    Vector2D get_velocity(double t);
    Vector2D get_acceleration(double t);
    double get_total_time();
    PosVelAcc get_values_at_time(double tt);
    std::vector<double> get_time_sections();
};

#endif // TRAJECTORY_H
