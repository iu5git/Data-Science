#include "quadropted_controller_cpp/controllers/trot_swing.hpp"

namespace quadropted {

TrotSwingController::TrotSwingController(int swing_ticks, double time_step, double z_leg_lift,
                                         Eigen::MatrixXd default_stance, int phase_length, int stance_ticks)
    : swing_ticks_(swing_ticks),
      time_step_(time_step),
      z_leg_lift_(z_leg_lift),
      default_stance_(std::move(default_stance)),
      phase_length_(phase_length),
      stance_ticks_(stance_ticks) {}

Eigen::Vector3d TrotSwingController::raibert_touchdown_location(int leg_index, const Eigen::Vector3d& cmd_vel) const {
    double scale_factor = 1.0;
    // Python: phase_length * time_step для delta_pos
    double total_time = phase_length_ * time_step_;
    Eigen::Vector3d delta_pos;
    delta_pos << cmd_vel.x() * total_time * scale_factor, cmd_vel.y() * total_time * scale_factor, 0.0;

    // Python: stance_ticks * time_step для yaw rotation
    double theta = stance_ticks_ * time_step_ * cmd_vel.z();
    Eigen::Matrix3d rotation = rotz(theta);

    return rotation * default_stance_.col(leg_index) + delta_pos;
}

double TrotSwingController::swing_height(double swing_prop) const {
    double scale_factor = 1.0;
    if (swing_prop < 0.5) {
        return (swing_prop / 0.5) * z_leg_lift_ * scale_factor;
    } else {
        return z_leg_lift_ * (1.0 - (swing_prop - 0.5) / 0.5) * scale_factor;
    }
}

Eigen::Vector3d TrotSwingController::next_foot_location(double swing_prop, int leg_index,
                                                        const Eigen::MatrixXd& current, const Eigen::Vector3d& cmd_vel,
                                                        double robot_height) const {
    assert(swing_prop >= 0.0 && swing_prop <= 1.0);

    Eigen::Vector3d foot_location = current.col(leg_index);
    double swing_h = swing_height(swing_prop);
    Eigen::Vector3d touchdown = raibert_touchdown_location(leg_index, cmd_vel);

    double time_left = time_step_ * swing_ticks_ * (1.0 - swing_prop);
    if (time_left < 1e-6) return touchdown;

    // Как в Python: velocity * XY_MASK — Z игнорируется
    Eigen::Vector3d velocity;
    velocity.x() = (touchdown.x() - foot_location.x()) / time_left;
    velocity.y() = (touchdown.y() - foot_location.y()) / time_left;
    velocity.z() = 0.0;

    Eigen::Vector3d delta_foot = velocity * time_step_;

    // Как в Python: foot_location * XY_MASK + z_vector + delta_foot
    // z_vector = [0, 0, swing_height + robot_height]
    Eigen::Vector3d result;
    result.x() = foot_location.x();
    result.y() = foot_location.y();
    result.z() = swing_h + robot_height;  // FIX: используем переданный robot_height
    result += delta_foot;

    return result;
}

}  // namespace quadropted
