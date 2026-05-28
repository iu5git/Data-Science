#include "quadropted_controller_cpp/controllers/crawl_stance.hpp"

namespace quadropted {

CrawlStanceController::CrawlStanceController(int phase_length, int stance_ticks, int swing_ticks, double time_step,
                                             double z_error_constant, double body_shift_y)
    : phase_length_(phase_length),
      stance_ticks_(stance_ticks),
      swing_ticks_(swing_ticks),
      time_step_(time_step),
      z_error_constant_(z_error_constant),
      body_shift_y_(body_shift_y) {}

Eigen::Vector3d CrawlStanceController::next_foot_location(int leg_index, const Eigen::MatrixXd& state_foot,
                                                          const Eigen::Vector3d& cmd_vel, double robot_height,
                                                          bool first_cycle, bool move_sideways, bool move_left) const {
    double z = state_foot(2, leg_index);

    double step_dist_x = cmd_vel.x() * (static_cast<double>(phase_length_) / swing_ticks_);
    int shift_factor = first_cycle ? 1 : 2;

    double side_vel = 0.0;
    if (move_sideways) {
        side_vel = move_left ? -(body_shift_y_ * shift_factor) / (time_step_ * stance_ticks_)
                             : (body_shift_y_ * shift_factor) / (time_step_ * stance_ticks_);
    }

    Eigen::Vector3d velocity;
    velocity.x() = -(step_dist_x / 3.0) / (time_step_ * stance_ticks_);
    velocity.y() = side_vel;
    velocity.z() = (1.0 / z_error_constant_) * (robot_height - z);

    Eigen::Vector3d delta_pos = velocity * time_step_;
    double yaw_delta = -cmd_vel.z() * time_step_;
    Eigen::Matrix3d delta_ori = rotz(yaw_delta);

    Eigen::Vector3d foot_location = state_foot.col(leg_index);
    return delta_ori * foot_location + delta_pos;
}

}  // namespace quadropted
