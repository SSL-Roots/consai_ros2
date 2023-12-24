#include "consai_robot_controller/trajectory/bangbangtrajectory1d.h"

#include <cmath>

BBTrajectoryPart::BBTrajectoryPart() : tEnd(0.0), acc(0.0), v0(0.0), s0(0.0)
{
}

BangBangTrajectory1D::BangBangTrajectory1D()
{
    for (int i = 0; i < MAX_PARTS; i++)
    {
        _parts[i] = BBTrajectoryPart();
    }
    _numParts = 0;
}

BangBangTrajectory1D::~BangBangTrajectory1D()
{
}

double BangBangTrajectory1D::get_position(double tt)
{
    double trajTime = std::max(0.0, tt);

    if (trajTime >= get_total_time())
    {
        BBTrajectoryPart lastPart = _parts[_numParts - 1];
        double t = lastPart.tEnd - _parts[_numParts - 2].tEnd;
        return lastPart.s0 + (lastPart.v0 * t) + (0.5 * lastPart.acc * t * t);
    }

    int pieceIdx = _find_part_idx(trajTime);
    BBTrajectoryPart piece = _parts[pieceIdx];
    double tPieceStart = 0.0;
    if (pieceIdx < 1)
    {
        tPieceStart = 0.0;
    }
    else
    {
        tPieceStart = _parts[pieceIdx - 1].tEnd;
    }
    double t = trajTime - tPieceStart;
    return piece.s0 + (piece.v0 * t) + (0.5 * piece.acc * t * t);
}

double BangBangTrajectory1D::get_position_mm(double tt)
{
    return get_position(tt) * 1000.0;
}

double BangBangTrajectory1D::get_velocity(double tt)
{
    double trajTime = std::max(0.0, tt);

    if (trajTime >= get_total_time())
    {
        return 0.0;
    }

    int pieceIdx = _find_part_idx(trajTime);
    BBTrajectoryPart piece = _parts[pieceIdx];
    double tPieceStart = 0.0;
    if (pieceIdx < 1)
    {
        tPieceStart = 0.0;
    }
    else
    {
        tPieceStart = _parts[pieceIdx - 1].tEnd;
    }
    double t = trajTime - tPieceStart;
    return piece.v0 + (piece.acc * t);
}

double BangBangTrajectory1D::get_acceleration(double tt)
{
    double trajTime = std::max(0.0, tt);

    if (trajTime >= get_total_time())
    {
        return 0.0;
    }

    return _find_part(trajTime).acc;
}

double BangBangTrajectory1D::get_total_time()
{
    return _parts[_numParts - 1].tEnd;
}


PosVelAcc BangBangTrajectory1D::get_values_at_time(double tt)
{
    double trajTime = std::max(0.0, tt);

    if (trajTime >= get_total_time())
    {
        return PosVelAcc(get_position(tt), 0.0, 0.0);
    }

    int pieceIdx = _find_part_idx(trajTime);
    BBTrajectoryPart piece = _parts[pieceIdx];
    double tPieceStart = 0.0;
    if (pieceIdx < 1)
    {
        tPieceStart = _parts[pieceIdx - 1].tEnd;
    }
    else
    {
        tPieceStart = 0.0;
    }
    double t = trajTime - tPieceStart;
    return PosVelAcc(
        piece.s0 + (piece.v0 * t) + (0.5 * piece.acc * t * t),
        piece.v0 + (piece.acc * t),
        piece.acc
    );
}


std::vector<double> BangBangTrajectory1D::get_time_sections()
{
    std::vector<double> sections;
    for (int i = 0; i < _numParts; i++)
    {
        sections.push_back(_parts[i].tEnd);
    }
    return sections;
}

double BangBangTrajectory1D::get_max_position()
{
    return _max_position;
}

double BangBangTrajectory1D::get_min_position()
{
    return _min_position;
}    

int BangBangTrajectory1D::_find_part_idx(double t)
{
    for (int i = 0; i < _numParts; i++)
    {
        if (t < _parts[i].tEnd)
        {
            return i;
        }
    }
    return _numParts - 1;
}

BBTrajectoryPart BangBangTrajectory1D::_find_part(double t)
{
    return _parts[_find_part_idx(t)];
}

void BangBangTrajectory1D::generate(double initialPos, double finalPos, double initialVel, double maxVel, double maxAcc)
{
    double x0 = initialPos;
    double xd0 = initialVel;
    double xTrg = finalPos;
    double xdMax = maxVel;
    double xddMax = maxAcc;
    double sAtZeroAcc = _vel_change_to_zero(x0, xd0, xddMax);

    if (sAtZeroAcc <= xTrg)
    {
        double sEnd = _vel_tri_to_zero(x0, xd0, xdMax, xddMax);

        if (sEnd >= xTrg)
        {
            _calc_tri(x0, xd0, xTrg, xddMax);
        }
        else
        {
            _calc_trapz(x0, xd0, xdMax, xTrg, xddMax);
        }
    }
    else
    {
        double sEnd = _vel_tri_to_zero(x0, xd0, -xdMax, xddMax);

        if (sEnd <= xTrg)
        {
            _calc_tri(x0, xd0, xTrg, -xddMax);
        }
        else
        {
            _calc_trapz(x0, xd0, -xdMax, xTrg, xddMax);
        }
    }

    _max_position = std::max(std::max(x0, sAtZeroAcc), xTrg);
    _min_position = std::min(std::min(x0, sAtZeroAcc), xTrg);
}
  
double BangBangTrajectory1D::_vel_change_to_zero(double s0, double v0, double aMax)
{
    if (v0 >= 0)
    {
        return s0 + (0.5 * v0 * v0 / aMax);
    }
    else
    {
        return s0 - (0.5 * v0 * v0 / aMax);
    }
}


double BangBangTrajectory1D::_vel_tri_to_zero(double s0, double v0, double v1, double aMax)
{
    if (v1 >= v0)
    {
        double a1 = aMax;
        double a2 = -aMax;
        double t1 = (v1 - v0) / a1;
        double s1 = s0 + (0.5 * (v0 + v1) * t1);
        double t2 = -v1 / a2;
        return s1 + (0.5 * v1 * t2);
    }
    else
    {
        double a1 = -aMax;
        double a2 = aMax;
        double t1 = (v1 - v0) / a1;
        double s1 = s0 + (0.5 * (v0 + v1) * t1);
        double t2 = -v1 / a2;
        return s1 + (0.5 * v1 * t2);
    }
}

void BangBangTrajectory1D::_calc_tri(double s0, double v0, double s2, double a)
{
    double sq;

    if (a > 0)
    {
        sq = ((a * (s2 - s0)) + (0.5 * v0 * v0)) / (a * a);
    }
    else
    {
        sq = ((-a * (s0 - s2)) + (0.5 * v0 * v0)) / (a * a);
    }

    double t2;
    if (sq > 0.0)
    {
        t2 = pow(sq, 0.5);
    }
    else
    {
        t2 = 0;
    }

    double v1 = a * t2;
    double t1 = (v1 - v0) / a;
    double s1 = s0 + ((v0 + v1) * 0.5 * t1);

    _parts[0].tEnd = t1;
    _parts[0].acc = a;
    _parts[0].v0 = v0;
    _parts[0].s0 = s0;
    _parts[1].tEnd = t1 + t2;
    _parts[1].acc = -a;
    _parts[1].v0 = v1;
    _parts[1].s0 = s1;
    _numParts = 2;
}

void BangBangTrajectory1D::_calc_trapz(double s0, double v0, double v1, double s3, double aMax)
{
    double a1, a3;
    if (v0 > v1)
    {
        a1 = -aMax;
    }
    else
    {
        a1 = aMax;
    }

    if (v1 > 0)
    {
        a3 = -aMax;
    }
    else
    {
        a3 = aMax;
    }

    double t1 = (v1 - v0) / a1;
    double v2 = v1;
    double t3 = -v2 / a3;

    double s1 = s0 + (0.5 * (v0 + v1) * t1);
    double s2 = s3 - (0.5 * v2 * t3);
    double t2 = (s2 - s1) / v1;

    _parts[0].tEnd = t1;
    _parts[0].acc = a1;
    _parts[0].v0 = v0;
    _parts[0].s0 = s0;
    _parts[1].tEnd = t1 + t2;
    _parts[1].acc = 0;
    _parts[1].v0 = v1;
    _parts[1].s0 = s1;
    _parts[2].tEnd = t1 + t2 + t3;
    _parts[2].acc = a3;
    _parts[2].v0 = v2;
    _parts[2].s0 = s2;
    _numParts = 3;
}

