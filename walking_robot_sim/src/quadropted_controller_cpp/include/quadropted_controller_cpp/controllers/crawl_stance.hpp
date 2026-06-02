#pragma once
#include <Eigen/Dense>

#include "quadropted_controller_cpp/utils/math_utils.hpp"

namespace quadropted {

class CrawlStanceController {
  public:
    CrawlStanceController(int phase_length, int stance_ticks, int swing_ticks, double time_step,
                          double z_error_constant, double body_shift_y);
    Eigen::Vector3d next_foot_location(int leg_index, const Eigen::MatrixXd& state_foot, const Eigen::Vector3d& cmd_vel,
                                       double robot_height, bool first_cycle, bool move_sideways, bool move_left) const;

  private:
    int phase_length_, stance_ticks_, swing_ticks_;
    double time_step_, z_error_constant_, body_shift_y_;
};

}  // namespace quadropted
