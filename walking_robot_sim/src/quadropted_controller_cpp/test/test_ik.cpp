#include <gtest/gtest.h>

#include "quadropted_controller_cpp/inverse_kinematics.hpp"

TEST(IK, compute_all_joint_angles) {
    // Python: [-0.608523, 0.061070, -1.630987, -2.533070, ...]
    Eigen::MatrixXd positions(3, 4);
    positions << 0.2, 0.2, -0.2, -0.2, -0.12, 0.12, -0.12, 0.12, -0.2, -0.2, -0.2, -0.2;

    auto angles = quadropted::compute_all_joint_angles(positions, 0.0, 0.0955, 0.213, 0.213);
    ASSERT_EQ(angles.size(), 12u);

    EXPECT_NEAR(angles[0], -0.608523, 1e-5);
    EXPECT_NEAR(angles[1], 0.061070, 1e-5);
    EXPECT_NEAR(angles[2], -1.630987, 1e-5);
    EXPECT_NEAR(angles[3], -2.533070, 1e-5);
}

TEST(IK, smoke_angles_in_realistic_range) {
    quadropted::InverseKinematics ik(0.3762, 0.0935, 0.0, 0.0955, 0.213, 0.213);
    Eigen::MatrixXd lp(3, 4);
    lp << 0.2, 0.2, -0.2, -0.2, -0.1, 0.1, -0.1, 0.1, 0, 0, 0, 0;
    auto angles = ik.inverse_kinematics(lp, 0, 0, 0, 0, 0, 0);
    for (double a : angles)
        EXPECT_LT(std::abs(a), 2 * M_PI + 0.1);
}

TEST(IK, smoke_exactly_12_angles) {
    quadropted::InverseKinematics ik(0.3762, 0.0935, 0.0, 0.0955, 0.213, 0.213);
    Eigen::MatrixXd lp(3, 4);
    lp << 0.2, 0.2, -0.2, -0.2, -0.1, 0.1, -0.1, 0.1, 0, 0, 0, 0;
    auto angles = ik.inverse_kinematics(lp, 0, 0, 0, 0, 0, 0);
    ASSERT_EQ(angles.size(), 12u);
}
