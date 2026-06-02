#include <gtest/gtest.h>

#include "quadropted_controller_cpp/odometry_state.hpp"
#include "quadropted_controller_cpp/odometry_update.hpp"

TEST(Odometry, append_delta_and_average) {
    quadropted::OdometryState state;  // пустой, не инициализированный
    state.filter_window_size = 5;
    for (double v : {0.1, 0.2, 0.3, 0.4, 0.5})
        state.append_delta(v, v * 2);

    auto [avg_x, avg_y] = state.average_delta();
    EXPECT_NEAR(avg_x, 0.3, 1e-10);
    EXPECT_NEAR(avg_y, 0.6, 1e-10);
    EXPECT_EQ(state.delta_x_queue.size(), 5u);
}

TEST(Odometry, reset) {
    quadropted::OdometryState state;
    state.x = 1.0;
    state.y = 2.0;
    state.theta = 0.5;
    state.delta_x_queue.push_back(0.1);
    state.reset();
    EXPECT_DOUBLE_EQ(state.x, 0.0);
    EXPECT_DOUBLE_EQ(state.y, 0.0);
    EXPECT_TRUE(state.delta_x_queue.empty());
}

TEST(Odometry, update_odometry) {
    quadropted::OdometryState state;
    state.linear_velocity_x = 0.02;
    state.linear_velocity_y = 0.01;
    state.theta = 0.1;
    state.foot_contacts = {false, false, false, false};

    quadropted::update_odometry(state, 0.02);
    EXPECT_NEAR(state.x, 0.000378, 1e-5);
    EXPECT_NEAR(state.y, 0.000239, 1e-5);
    EXPECT_NEAR(state.theta, 0.1, 1e-10);
}
