#include <gtest/gtest.h>

#include <cmath>

#include "quadropted_controller_cpp/rotation_matrices.hpp"

// ════════════════════════════════════════════════════════════
// Base Link Roll Tests — критические тесты для проверки крена
// ════════════════════════════════════════════════════════════

// Тест 1: rotxyz(0,0,0) даёт identity
TEST(BaseLinkRoll, rotxyz_zero_is_identity) {
    auto m = quadropted::rotxyz(0.0, 0.0, 0.0);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_NEAR(m(i, j), (i == j ? 1.0 : 0.0), 1e-10);
        }
    }
}

// Тест 2: rotx(π/4) — корректность roll = 45°
TEST(BaseLinkRoll, rotx_45_degrees) {
    double angle = M_PI / 4.0;  // 45 градусов
    auto m = quadropted::rotx(angle);
    double c = std::cos(angle);
    double s = std::sin(angle);

    EXPECT_NEAR(m(0, 0), 1.0, 1e-10);
    EXPECT_NEAR(m(0, 1), 0.0, 1e-10);
    EXPECT_NEAR(m(0, 2), 0.0, 1e-10);
    EXPECT_NEAR(m(1, 1), c, 1e-10);
    EXPECT_NEAR(m(1, 2), -s, 1e-10);
    EXPECT_NEAR(m(2, 1), s, 1e-10);
    EXPECT_NEAR(m(2, 2), c, 1e-10);
}

// Тест 3: rotxyz(π/4, 0, 0) — только roll 45°
TEST(BaseLinkRoll, rotxyz_only_roll_45) {
    double angle = M_PI / 4.0;
    auto m = quadropted::rotxyz(angle, 0.0, 0.0);
    auto expected = quadropted::rotx(angle);

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_NEAR(m(i, j), expected(i, j), 1e-10);
        }
    }
}

// Тест 4: rotxyz(roll, pitch, yaw) совпадает с Python для 10 наборов углов
TEST(BaseLinkRoll, rotxyz_matches_python_multiple_angles) {
    struct TestCase {
        double roll, pitch, yaw;
        double expected[3][3];
    };

    // Значения из Python: rotxyz(roll, pitch, yaw)
    TestCase cases[] = {
        {0.0, 0.0, 0.0, {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}},
        {0.1,
         -0.05,
         0.02,
         {{0.998551, -0.019974, -0.049979}, {0.014910, 0.994905, -0.099709}, {0.051716, 0.098819, 0.993761}}},
        {M_PI / 4, 0.0, 0.0, {{1, 0, 0}, {0, 0.707107, -0.707107}, {0, 0.707107, 0.707107}}},
        {0.0, M_PI / 4, 0.0, {{0.707107, 0, 0.707107}, {0, 1, 0}, {-0.707107, 0, 0.707107}}},
        {0.0, 0.0, M_PI / 4, {{0.707107, -0.707107, 0}, {0.707107, 0.707107, 0}, {0, 0, 1}}},
        {M_PI / 6,
         M_PI / 4,
         M_PI / 3,
         {{0.353553, -0.573223, 0.739199}, {0.612372, 0.739199, 0.280330}, {-0.707107, 0.353553, 0.612372}}},
        {-0.5,
         0.3,
         -0.2,
         {{0.955336, 0.198669, -0.219217}, {-0.074723, 0.890871, -0.448171}, {0.286193, -0.410731, -0.865473}}},
        {M_PI / 2, 0.0, 0.0, {{1, 0, 0}, {0, 0, -1}, {0, 1, 0}}},
        {0.0, -M_PI / 2, 0.0, {{0, 0, -1}, {0, 1, 0}, {1, 0, 0}}},
        {M_PI / 4, M_PI / 4, 0.0, {{0.707107, 0, 0.707107}, {0.5, 0.707107, -0.5}, {-0.5, 0.707107, 0.5}}},
    };

    for (const auto& tc : cases) {
        auto m = quadropted::rotxyz(tc.roll, tc.pitch, tc.yaw);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                EXPECT_NEAR(m(i, j), tc.expected[i][j], 1e-5) << "Mismatch at roll=" << tc.roll << " pitch=" << tc.pitch
                                                              << " yaw=" << tc.yaw << " [" << i << "," << j << "]";
            }
        }
    }
}

// Тест 5: R_legs матрица корректна: rotxyz(π/2, -π/2, 0)
TEST(BaseLinkRoll, R_legs_matrix_correct) {
    auto R = quadropted::rotxyz(M_PI / 2, -M_PI / 2, 0.0);

    // Ожидаемая матрица из Python local_positions.py
    EXPECT_NEAR(R(0, 0), 0.0, 1e-10);
    EXPECT_NEAR(R(0, 1), 0.0, 1e-10);
    EXPECT_NEAR(R(0, 2), -1.0, 1e-10);
    EXPECT_NEAR(R(1, 0), -1.0, 1e-10);
    EXPECT_NEAR(R(1, 1), 0.0, 1e-10);
    EXPECT_NEAR(R(1, 2), 0.0, 1e-10);
    EXPECT_NEAR(R(2, 0), 0.0, 1e-10);
    EXPECT_NEAR(R(2, 1), 1.0, 1e-10);
    EXPECT_NEAR(R(2, 2), 0.0, 1e-10);
}

// Тест 6: rotxyz — ортогональность матрицы (R * R^T = I)
TEST(BaseLinkRoll, rotxyz_orthogonal) {
    auto R = quadropted::rotxyz(0.3, -0.5, 0.7);
    auto I = R * R.transpose();

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_NEAR(I(i, j), (i == j ? 1.0 : 0.0), 1e-10);
        }
    }
}

// Тест 7: rotxyz — определитель = 1 (proper rotation)
TEST(BaseLinkRoll, rotxyz_determinant_is_one) {
    auto R = quadropted::rotxyz(0.3, -0.5, 0.7);
    double det = R.determinant();
    EXPECT_NEAR(det, 1.0, 1e-10);
}

// Тест 8: rotxyz(π/4, 0, 0) * [0, 1, 0]^T = [0, cos(π/4), sin(π/4)]^T
TEST(BaseLinkRoll, roll_45_transforms_y_axis) {
    double angle = M_PI / 4.0;
    auto R = quadropted::rotxyz(angle, 0.0, 0.0);
    Eigen::Vector3d y_axis(0.0, 1.0, 0.0);
    Eigen::Vector3d result = R * y_axis;

    EXPECT_NEAR(result.x(), 0.0, 1e-10);
    EXPECT_NEAR(result.y(), std::cos(angle), 1e-10);
    EXPECT_NEAR(result.z(), std::sin(angle), 1e-10);
}

// Тест 9: rotxyz(π/4, 0, 0) * [0, 0, 1]^T = [0, -sin(π/4), cos(π/4)]^T
TEST(BaseLinkRoll, roll_45_transforms_z_axis) {
    double angle = M_PI / 4.0;
    auto R = quadropted::rotxyz(angle, 0.0, 0.0);
    Eigen::Vector3d z_axis(0.0, 0.0, 1.0);
    Eigen::Vector3d result = R * z_axis;

    EXPECT_NEAR(result.x(), 0.0, 1e-10);
    EXPECT_NEAR(result.y(), -std::sin(angle), 1e-10);
    EXPECT_NEAR(result.z(), std::cos(angle), 1e-10);
}

// Тест 10: rotxyz(-π/4, 0, 0) — обратный roll
TEST(BaseLinkRoll, rotxyz_negative_roll_45) {
    double angle = -M_PI / 4.0;
    auto m = quadropted::rotxyz(angle, 0.0, 0.0);
    double c = std::cos(angle);
    double s = std::sin(angle);

    EXPECT_NEAR(m(0, 0), 1.0, 1e-10);
    EXPECT_NEAR(m(1, 1), c, 1e-10);
    EXPECT_NEAR(m(1, 2), -s, 1e-10);
    EXPECT_NEAR(m(2, 1), s, 1e-10);
    EXPECT_NEAR(m(2, 2), c, 1e-10);
}
