#include "quadropted_controller_cpp/controllers/crawl_swing.hpp"

namespace quadropted {

CrawlSwingController::CrawlSwingController(int swing_ticks, double time_step, double z_leg_lift,
                                           Eigen::MatrixXd default_stance, int phase_length, int stance_ticks,
                                           double body_shift_y)
    : swing_ticks_(swing_ticks),
      time_step_(time_step),
      z_leg_lift_(z_leg_lift),
      default_stance_(std::move(default_stance)),
      phase_length_(phase_length),
      stance_ticks_(stance_ticks),
      body_shift_y_(body_shift_y) {}

Eigen::Vector3d CrawlSwingController::raibert_touchdown_location(int leg_index, const Eigen::Vector3d& cmd_vel,
                                                                 bool shifted_left) const {
    // Python: delta_pos_2d = command.velocity * phase_length * time_step
    double total_time = phase_length_ * time_step_;
    Eigen::Vector3d delta_pos;
    delta_pos << cmd_vel.x() * total_time, cmd_vel.y() * total_time, 0.0;

    // Python: theta = stance_ticks * time_step * command.yaw_rate
    double theta = stance_ticks_ * time_step_ * cmd_vel.z();
    Eigen::Matrix3d rotation = rotz(theta);

    // Python: shift_correction[1] = -body_shift_y if shifted_left else body_shift_y
    Eigen::Vector3d shift_correction;
    shift_correction << 0.0, (shifted_left ? -body_shift_y_ : body_shift_y_), 0.0;

    return rotation * default_stance_.col(leg_index) + delta_pos + shift_correction;
}

double CrawlSwingController::swing_height(double swing_prop) const {
    if (swing_prop < 0.5) {
        return (swing_prop / 0.5) * z_leg_lift_;
    } else {
        return z_leg_lift_ * (1.0 - (swing_prop - 0.5) / 0.5);
    }
}

Eigen::Vector3d CrawlSwingController::next_foot_location(double swing_prop, int leg_index,
                                                         const Eigen::MatrixXd& current, const Eigen::Vector3d& cmd_vel,
                                                         double robot_height) const {
    assert(swing_prop >= 0.0 && swing_prop <= 1.0);

    // Python: foot_location = state.foot_locations[:, leg_index]
    Eigen::Vector3d foot_location = current.col(leg_index);

    double swing_h = swing_height(swing_prop);

    // Python: shifted_left = (phase_index in (1,3)) — определяется из crawl_gait
    // Для совместимости: shifted_left передаётся отдельно из crawl_gait step
    bool shifted_left = false;  // заглушка, crawl_gait должен определить
    (void)shifted_left;         // TODO: передать phase_index из crawl_gait

    Eigen::Vector3d touchdown = raibert_touchdown_location(leg_index, cmd_vel, shifted_left);

    double time_left = time_step_ * swing_ticks_ * (1.0 - swing_prop);
    if (time_left < 1e-6) return touchdown;

    // Python: velocity * np.array([1, 1, 0]) — XY mask
    Eigen::Vector3d velocity = (touchdown - foot_location) / time_left;
    velocity.z() = 0.0;

    Eigen::Vector3d delta_foot = velocity * time_step_;

    // Python: z_vector = [0, 0, swing_height_ + command.robot_height]
    Eigen::Vector3d z_vector;
    z_vector << 0.0, 0.0, swing_h + robot_height;

    // Python: foot_location * [1,1,0] + z_vector + delta_foot
    return Eigen::Vector3d(foot_location.x(), foot_location.y(), 0.0) + z_vector + delta_foot;
}

}  // namespace quadropted
