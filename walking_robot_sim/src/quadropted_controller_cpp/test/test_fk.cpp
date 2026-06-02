#include <gtest/gtest.h>

#include "quadropted_controller_cpp/forward_kinematics.hpp"

TEST(FK, forward_kinematics_all_legs) {
    // angles = [0, 0.3, -0.6] * 4
    // Python: [[0.686308, 0.046750, 0.097669], [0.686308, -0.046750, 0.097669],
    //          [0.310108, 0.046750, 0.097669], [0.310108, -0.046750, 0.097669]]
    quadropted::ForwardKinematics fk(0.3762, 0.0935, 0.0, 0.0955, 0.213, 0.213);
    std::vector<double> angles = {0, 0.3, -0.6, 0, 0.3, -0.6, 0, 0.3, -0.6, 0, 0.3, -0.6};
    auto result = fk.forward_kinematics_all_legs(angles);

    EXPECT_NEAR(result[0].x(), 0.686308, 1e-5);
    EXPECT_NEAR(result[0].y(), 0.046750, 1e-5);
    EXPECT_NEAR(result[0].z(), 0.097669, 1e-5);

    EXPECT_NEAR(result[1].x(), 0.686308, 1e-5);
    EXPECT_NEAR(result[1].y(), -0.046750, 1e-5);

    EXPECT_NEAR(result[2].x(), 0.310108, 1e-5);
    EXPECT_NEAR(result[3].x(), 0.310108, 1e-5);
}

TEST(FK, smoke_4_positions_3_coords) {
    quadropted::ForwardKinematics fk(0.3762, 0.0935, 0.0, 0.0955, 0.213, 0.213);
    std::vector<double> angles(12, 0.0);
    auto result = fk.forward_kinematics_all_legs(angles);
    ASSERT_EQ(result.size(), 4u);
    for (auto& p : result)
        ASSERT_EQ(p.size(), 3);
}
