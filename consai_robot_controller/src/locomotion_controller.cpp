#include "consai_robot_controller/locomotion_controller.hpp"


LocomotionController::LocomotionController(_Float64 kp_xy, _Float64 kp_theta, uint64_t dt_ms, double max_linear_velocity, double max_angular_velocity, double max_linear_acceleration, double max_angular_acceleration) {
    this->trajectory_follow_controller_ = TrajectoryFollowController(kp_xy, dt_ms);
    this->target_velocity_ = Velocity2D(0, 0, 0);
    this->output_velocity_ = Velocity2D(0, 0, 0);
    this->state_ = INITIALIZED;    
    this->kp_xy = kp_xy;
    this->kp_theta = kp_theta;
    this->dt_ms_ = dt_ms;
    this->max_linear_velocity_ = max_linear_velocity;
    this->max_angular_velocity_ = max_angular_velocity;
    this->max_linear_acceleration_ = max_linear_acceleration;
    this->max_angular_acceleration_ = max_angular_acceleration;
}

LocomotionController::ControllerState LocomotionController::moveConstantVelocity(const Velocity2D& velocity) {
    // 一定速度での移動を指示するメソッドの実装
    target_velocity_ = Velocity2D(velocity.x, velocity.y, velocity.theta);

    state_ = RUNNING_CONSTANT_VELOCITY;
    return state_;
}

LocomotionController::ControllerState LocomotionController::moveToPose(const Pose2D& goal_pose, const Pose2D& current_pose) {
    // 特定のポーズへの移動を指示するメソッドの実装

    // 軌道生成を行う
    State2D current_state = State2D(current_pose, this->output_velocity_);
    State2D goal_state = State2D(goal_pose, Velocity2D(0, 0, 0));
    Trajectory trajectory = TrajectoryGenerator::generate(current_state, goal_state, this->max_linear_velocity_, this->max_linear_acceleration_, 0.1);
    trajectory_follow_controller_.initialize(std::make_shared<Trajectory>(trajectory));

    state_ = RUNNING_FOLLOW_TRAJECTORY;
    return state_;
}

std::pair<Velocity2D, LocomotionController::ControllerState> LocomotionController::run(const State2D& current_state) {
    // 現在の状態から次のステップの速度と状態を計算するメソッドの実装
    Velocity2D output_velocity;

    switch (this->state_) {
        case RUNNING_CONSTANT_VELOCITY:
            // 一定速度での移動を継続
            output_velocity = target_velocity_;
            break;

        case GENERATING_TRAJECTORY:
            // 軌道生成を実行
            break;

        case RUNNING_FOLLOW_TRAJECTORY:
            // 軌道追従制御を実行
            output_velocity = runFollowTrajectory(current_state);
            break;

        case COMPLETE:
            // 完了
            output_velocity = Velocity2D(0, 0, 0);
            break;

        default:
            // 未定義の状態
            output_velocity = Velocity2D(0, 0, 0);
            break;
    }

    // TODO: 速度・加速度リミットをここに移植する
    this->output_velocity_ = output_velocity;
    return std::make_pair(output_velocity, state_);
}

LocomotionController::ControllerState LocomotionController::getState() {
    // コントローラの状態を取得するメソッドの実装
    return state_;
}

/**
 * Private
*/
Velocity2D LocomotionController::runFollowTrajectory(const State2D& current_state) {
    auto control_output = trajectory_follow_controller_.run(current_state);
    Velocity2D output = control_output.first;
    if (control_output.second == TrajectoryFollowController::ControllerState::COMPLETE) {
        this->state_ = COMPLETE;
    }

    return output;
}