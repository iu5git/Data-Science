#include <gtest/gtest.h>

#include <cmath>

#include "quadropted_controller_cpp/rotation_matrices.hpp"

TEST(RotMatrices, rotx) {
    auto m = quadropted::rotx(M_PI / 2);
    EXPECT_NEAR(m(1, 1), 0, 1e-10);
    EXPECT_NEAR(m(1, 2), -1, 1e-10);
    EXPECT_NEAR(m(2, 1), 1, 1e-10);
}

TEST(RotMatrices, roty) {
    auto m = quadropted::roty(0.3);
    EXPECT_NEAR(m(0, 0), std::cos(0.3), 1e-10);
    EXPECT_NEAR(m(0, 2), std::sin(0.3), 1e-10);
}

TEST(RotMatrices, rotz) {
    auto m = quadropted::rotz(0.7);
    EXPECT_NEAR(m(0, 0), std::cos(0.7), 1e-10);
    EXPECT_NEAR(m(1, 0), std::sin(0.7), 1e-10);
}

TEST(RotMatrices, rotxyz_matches_python) {
    // Значения из Python: rotxyz(0.1, -0.05, 0.02)
    // → [[0.998551, -0.019974, -0.049979],
    //    [0.014910,  0.994905, -0.099709],
    //    [0.051716,  0.098819,  0.993761]]
    auto m = quadropted::rotxyz(0.1, -0.05, 0.02);
    EXPECT_NEAR(m(0, 0), 0.998551, 1e-5);
    EXPECT_NEAR(m(0, 1), -0.019974, 1e-5);
    EXPECT_NEAR(m(0, 2), -0.049979, 1e-5);
    EXPECT_NEAR(m(1, 0), 0.014910, 1e-5);
    EXPECT_NEAR(m(1, 1), 0.994905, 1e-5);
    EXPECT_NEAR(m(1, 2), -0.099709, 1e-5);
    EXPECT_NEAR(m(2, 0), 0.051716, 1e-5);
    EXPECT_NEAR(m(2, 1), 0.098819, 1e-5);
    EXPECT_NEAR(m(2, 2), 0.993761, 1e-5);
}
