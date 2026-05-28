#include "quadropted_controller_cpp/controllers/trot_gait.hpp"

#include <cmath>

namespace quadropted {

TrotGaitController::TrotGaitController(double stance_time, double swing_time, double time_step, bool use_imu,
                                       Eigen::MatrixXd default_stance)
    : GaitController(stance_time, swing_time, time_step,
                     (Eigen::MatrixXi(4, 4) << 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0).finished(),
                     default_stance),
      use_imu_(use_imu),
      swing_(static_cast<int>(swing_time / time_step), time_step, 0.14, default_stance, phase_length(),
             static_cast<int>(stance_time / time_step)),
      stance_(phase_length(), stance_ticks(), swing_ticks(), time_step, 0.02),
      pid_(0.15, 0.02, 0.002) {}

Eigen::MatrixXd TrotGaitController::step(int ticks, const Eigen::MatrixXd& current, const Eigen::Vector3d& cmd_vel,
                                         double robot_height) const {
    Eigen::MatrixXd next = current;
    for (int leg = 0; leg < 4; ++leg) {
        auto contacts_vec = contacts(ticks);
        int sub = subphase_ticks(ticks);
        if (contacts_vec(leg) == 1) {
            next.col(leg) = stance_.next_foot_location(leg, current, cmd_vel, robot_height);
        } else {
            double swing_prop = static_cast<double>(sub) / swing_ticks_;
            next.col(leg) = swing_.next_foot_location(swing_prop, leg, current, cmd_vel, robot_height);
        }
    }
    return next;
}

}  // namespace quadropted
