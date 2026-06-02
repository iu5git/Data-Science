#include <gtest/gtest.h>

#include <cmath>

#include "quadropted_controller_cpp/homogeneous_transforms.hpp"

TEST(HomogTransform, transxyz) {
    auto m = quadropted::homog_transxyz(0.1, 0.2, 0.3);
    EXPECT_NEAR(m(0, 3), 0.1, 1e-10);
    EXPECT_NEAR(m(1, 3), 0.2, 1e-10);
    EXPECT_NEAR(m(2, 3), 0.3, 1e-10);
    EXPECT_NEAR(m(3, 3), 1.0, 1e-10);
}

TEST(HomogTransform, transform) {
    auto m = quadropted::homog_transform(0.1, 0.2, 0.3, 0.4, 0.5, 0.6);
    EXPECT_NEAR(m(0, 3), 0.1, 1e-10);
    EXPECT_NEAR(m(3, 3), 1.0, 1e-10);
}

TEST(HomogTransform, inverse) {
    Eigen::Matrix4d m;
    m << 0, 0, 1, 0.1, 0, 1, 0, 0.2, -1, 0, 0, 0.3, 0, 0, 0, 1;
    auto inv = quadropted::homog_transform_inverse(m);
    // Проверка: M @ inv ≈ I
    auto product = m * inv;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            EXPECT_NEAR(product(i, j), (i == j ? 1.0 : 0.0), 1e-10);
}
