#include "quadropted_controller_cpp/kinematics/forward_kinematics.hpp"

#include <cmath>

namespace quadropted {

Eigen::Vector2d LegBasePositions::get(int leg_index, double body_length, double body_width) {
    double hl = body_length / 2.0;
    double hw = body_width / 2.0;
    switch (leg_index) {
        case 0:
            return {hl, hw};  // FR
        case 1:
            return {hl, -hw};  // FL
        case 2:
            return {-hl, hw};  // RR
        case 3:
            return {-hl, -hw};  // RL
        default:
            throw std::invalid_argument("Invalid leg_index. Must be 0 (FR), 1 (FL), 2 (RR), or 3 (RL).");
    }
}

Eigen::Vector3d compute_leg_fk_chain(double theta_hip, double theta_thigh, double theta_calf, double base_x,
                                     double base_y, double l1, double l2, double l3, double l4) {
    auto build_homog_transform = [](double dx, double dy, double dz, double alpha, double beta, double gamma) {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = rotxyz(alpha, beta, gamma);
        T(0, 3) = dx;
        T(1, 3) = dy;
        T(2, 3) = dz;
        return T;
    };

    Eigen::Matrix4d T_base = build_homog_transform(base_x, base_y, -l1, 0, 0, 0);
    Eigen::Matrix4d T_hip = build_homog_transform(0, 0, 0, 0, 0, theta_hip);
    Eigen::Matrix4d T_thigh = build_homog_transform(0, 0, 0, 0, theta_thigh, 0);
    Eigen::Matrix4d T_thigh_t = build_homog_transform(l2, 0, 0, 0, 0, 0);
    Eigen::Matrix4d T_calf = build_homog_transform(0, 0, 0, 0, theta_calf, 0);
    Eigen::Matrix4d T_calf_t = build_homog_transform(l3, 0, 0, 0, 0, 0);
    Eigen::Matrix4d T_foot = build_homog_transform(l4, 0, 0, 0, 0, 0);

    Eigen::Matrix4d T_total = T_base * T_hip * T_thigh * T_thigh_t * T_calf * T_calf_t * T_foot;

    Eigen::Vector4d foot_hom = T_total * Eigen::Vector4d(0, 0, 0, 1);
    return foot_hom.head<3>();
}

ForwardKinematics::ForwardKinematics(double body_length, double body_width, double l1, double l2, double l3, double l4)
    : body_length_(body_length), body_width_(body_width), l1_(l1), l2_(l2), l3_(l3), l4_(l4) {}

std::vector<Eigen::Vector3d> ForwardKinematics::forward_kinematics_all_legs(
    const std::vector<double>& joint_angles) const {
    if (joint_angles.size() != 12) {
        throw std::invalid_argument("Expected 12 joint angles.");
    }

    std::vector<Eigen::Vector3d> foot_positions;
    foot_positions.reserve(4);

    for (int leg = 0; leg < 4; ++leg) {
        int idx = leg * 3;
        double theta_hip = joint_angles[idx];
        double theta_thigh = joint_angles[idx + 1];
        double theta_calf = joint_angles[idx + 2];

        Eigen::Vector2d base = LegBasePositions::get(leg, body_length_, body_width_);
        foot_positions.push_back(
            compute_leg_fk_chain(theta_hip, theta_thigh, theta_calf, base.x(), base.y(), l1_, l2_, l3_, l4_));
    }

    return foot_positions;
}

}  // namespace quadropted
