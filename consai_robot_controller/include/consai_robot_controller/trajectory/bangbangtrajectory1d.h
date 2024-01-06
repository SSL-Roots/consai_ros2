// bangbangtrajectory1d.h
#ifndef BANGBANGTRAJECTORY1D_H
#define BANGBANGTRAJECTORY1D_H

#include "consai_robot_controller/trajectory/utils.h"
#include "consai_robot_controller/trajectory/trajectory.h"

class BBTrajectoryPart
{
public:
  BBTrajectoryPart();
  double tEnd;
  double acc;
  double v0;
  double s0;
};

class BangBangTrajectory1D
{
public:
  static const int MAX_PARTS = 3;

  BangBangTrajectory1D();
  ~BangBangTrajectory1D();

  double get_position(double tt);
  double get_position_mm(double tt);
  double get_velocity(double tt);
  double get_acceleration(double tt);
  double  get_total_time();
  void generate(
    double initialPos, double finalPos, double initialVel, double maxVel,
    double maxAcc);
  PosVelAcc get_values_at_time(double tt);
  std::vector < double > get_time_sections();
  double get_max_position();
  double get_min_position();

private:
  BBTrajectoryPart _parts[MAX_PARTS];
  int _numParts;
  double _max_position;
  double _min_position;

  int _find_part_idx(double t);
  BBTrajectoryPart _find_part(double t);
  void _calc_tri();
  double _vel_change_to_zero(double s0, double v0, double aMax);
  double _vel_tri_to_zero(double s0, double v0, double v1, double aMax);
  void _calc_tri(double s0, double v0, double s2, double a);
  void _calc_trapz(double s0, double v0, double v1, double s3, double aMax);


};


# endif // BANGBANGTRAJECTORY1D_H
