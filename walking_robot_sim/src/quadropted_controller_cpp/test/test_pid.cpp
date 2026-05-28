#include <gtest/gtest.h>

#include "quadropted_controller_cpp/pid_controller.hpp"

TEST(PID, run_returns_2_elements) {
    quadropted::PIDController pid(0.15, 0.02, 0.002);
    pid.reset(0.0);
    auto result = pid.run(0.1, -0.05, 0.02);
    ASSERT_EQ(result.size(), 2u);
}
