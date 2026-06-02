#pragma once
#include <Eigen/Dense>
#include <array>
#include <vector>

#include "quadropted_controller_cpp/kinematics/forward_kinematics.hpp"

namespace quadropted {

Eigen::MatrixXd compute_local_positions(const Eigen::MatrixXd& leg_positions, double body_length, double body_width,
                                        double dx, double dy, double dz, double roll, double pitch, double yaw);

std::array<double, 3> compute_joint_angles_for_leg(double x, double y, double z, int leg_index, double l1, double l2,
                                                   double l3, double l4);

std::vector<double> compute_all_joint_angles(const Eigen::MatrixXd& positions, double l1, double l2, double l3,
                                             double l4);

class InverseKinematics {
  public:
    InverseKinematics(double body_length, double body_width, double l1, double l2, double l3, double l4);

    Eigen::MatrixXd get_local_positions(const Eigen::MatrixXd& leg_positions, double dx, double dy, double dz,
                                        double roll, double pitch, double yaw) const;

    std::vector<double> inverse_kinematics(const Eigen::MatrixXd& leg_positions, double dx, double dy, double dz,
                                           double roll, double pitch, double yaw) const;

  private:
    ForwardKinematics fk_;
    double body_length_, body_width_, l1_, l2_, l3_, l4_;
};

}  // namespace quadropted
