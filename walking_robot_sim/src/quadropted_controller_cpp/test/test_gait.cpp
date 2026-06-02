#include <gtest/gtest.h>

#include "quadropted_controller_cpp/gait_controller.hpp"

TEST(Gait, phase_ticks_has_4_elements) {
    Eigen::MatrixXi cp(4, 4);
    cp << 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0;
    quadropted::GaitController gc(0.04, 0.18, 0.02, cp, Eigen::MatrixXd::Zero(3, 4));
    const auto& pt = gc.phase_ticks();
    ASSERT_EQ(pt.size(), 4u);
    EXPECT_EQ(pt[0], 2);
    EXPECT_EQ(pt[1], 9);
    EXPECT_EQ(pt[2], 2);
    EXPECT_EQ(pt[3], 9);
}

TEST(Gait, contacts_has_4_elements) {
    Eigen::MatrixXi cp(4, 4);
    cp << 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0;
    quadropted::GaitController gc(0.04, 0.18, 0.02, cp, Eigen::MatrixXd::Zero(3, 4));
    auto c = gc.contacts(0);
    ASSERT_EQ(c.size(), 4);
}
