#pragma once
#include <Eigen/Dense>
#include <array>
#include <vector>

#include "quadropted_controller_cpp/utils/math_utils.hpp"

namespace quadropted {

struct LegBasePositions {
    static Eigen::Vector2d get(int leg_index, double body_length, double body_width);
};

Eigen::Vector3d compute_leg_fk_chain(double theta_hip, double theta_thigh, double theta_calf, double base_x,
                                     double base_y, double l1, double l2, double l3, double l4);

class ForwardKinematics {
  public:
    ForwardKinematics(double body_length, double body_width, double l1, double l2, double l3, double l4);
    std::vector<Eigen::Vector3d> forward_kinematics_all_legs(const std::vector<double>& joint_angles) const;

  private:
    double body_length_, body_width_, l1_, l2_, l3_, l4_;
};

}  // namespace quadropted
