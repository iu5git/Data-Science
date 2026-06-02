#include "quadropted_controller_cpp/controllers/trot_stance.hpp"

#include "quadropted_controller_cpp/utils/rotation_matrices.hpp"

namespace quadropted {

TrotStanceController::TrotStanceController(int phase_length, int stance_ticks, int swing_ticks, double time_step,
                                           double z_error_constant)
    : phase_length_(phase_length),
      stance_ticks_(stance_ticks),
      swing_ticks_(swing_ticks),
      time_step_(time_step),
      z_error_constant_(z_error_constant) {}

Eigen::Vector3d TrotStanceController::position_delta(int leg_index, const Eigen::MatrixXd& state_foot,
                                                     const Eigen::Vector3d& cmd_vel, double robot_height) const {
    double z = state_foot(2, leg_index);  // FIX: использовать leg_index вместо 0

    double step_dist_x = cmd_vel.x() * (static_cast<double>(phase_length_) / swing_ticks_);
    double step_dist_y = cmd_vel.y() * (static_cast<double>(phase_length_) / swing_ticks_);

    Eigen::Vector3d velocity;
    velocity.x() = -(step_dist_x / 4.0) / (time_step_ * stance_ticks_);
    velocity.y() = -(step_dist_y / 4.0) / (time_step_ * stance_ticks_);
    velocity.z() = (1.0 / z_error_constant_) * (robot_height - z);

    Eigen::Vector3d delta_pos = velocity * time_step_;
    return delta_pos;
}

Eigen::Vector3d TrotStanceController::next_foot_location(int leg_index, const Eigen::MatrixXd& state_foot,
                                                         const Eigen::Vector3d& cmd_vel, double robot_height) const {
    Eigen::Vector3d foot_location = state_foot.col(leg_index);
    Eigen::Vector3d delta_pos = position_delta(leg_index, state_foot, cmd_vel, robot_height);

    // FIX: rotxyz(roll, pitch, yaw) — как в Python trot_stance.py
    // cmd_vel = [roll_rate, pitch_rate, yaw_rate]
    Eigen::Matrix3d delta_ori = rotxyz(-cmd_vel.x() * time_step_, -cmd_vel.y() * time_step_, -cmd_vel.z() * time_step_);

    return delta_ori * foot_location + delta_pos;
}

}  // namespace quadropted
