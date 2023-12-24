#include "consai_robot_controller/trajectory_follow_control.hpp"

class TrajectoryGenerator {
public:
    static Trajectory generate(
        State2D start_state, State2D end_state, double vmax, double acc, double accuracy
    );
};
