#include <gtest/gtest.h>

#include <cmath>

#include "quadropted_controller_cpp/message_builders.hpp"

TEST(MessageBuilders, build_odometry_data_has_keys) {
    auto data = quadropted::build_odometry_data(1.0, 2.0, 0.5, 0.1, 0.05, 0.02, "odom", "base", "now");
    EXPECT_NEAR(data.pose_position.x, 1.0, 1e-10);
    EXPECT_NEAR(data.pose_position.y, 2.0, 1e-10);
    EXPECT_NEAR(data.twist_linear.x, 0.1, 1e-10);
    EXPECT_NEAR(data.twist_angular.z, 0.02, 1e-10);
}

TEST(MessageBuilders, build_tf_data_has_keys) {
    auto data = quadropted::build_tf_data(1.0, 2.0, 0.5, "odom", "base", "now");
    EXPECT_EQ(data.header_frame_id, "odom");
    EXPECT_EQ(data.child_frame_id, "base");
    EXPECT_NEAR(data.translation.x, 1.0, 1e-10);
    EXPECT_NEAR(data.translation.y, 2.0, 1e-10);
    EXPECT_NEAR(data.rotation.z, std::sin(0.25), 1e-5);
}
