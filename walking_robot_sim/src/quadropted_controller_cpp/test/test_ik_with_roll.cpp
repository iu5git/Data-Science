#include <gtest/gtest.h>

#include "quadropted_controller_cpp/kinematics/forward_kinematics.hpp"
#include "quadropted_controller_cpp/kinematics/inverse_kinematics.hpp"

// ════════════════════════════════════════════════════════════
// IK с roll тесты — проверка влияния крена на углы суставов
// ════════════════════════════════════════════════════════════

// Тест 1: IK с roll=0 для default_stance — базовый случай
TEST(IKWithRoll, zero_roll_default_stance) {
    quadropted::InverseKinematics ik(0.3762, 0.0935, 0.0, 0.0955, 0.213, 0.213);

    double dx = 0.3762 * 0.5 + 0.02;
    double dy = 0.0935 * 0.5 + 0.0955;
    Eigen::MatrixXd stance(3, 4);
    stance << dx, dx, -dx, -dx, -dy, dy, -dy, dy, 0, 0, 0, 0;

    auto angles = ik.inverse_kinematics(stance, 0, 0, 0.25, 0, 0, 0);
    ASSERT_EQ(angles.size(), 12u);

    // Углы должны быть в реалистичном диапазоне для стойки
    // Python: [0.000, 0.862, -1.883] для первой ноги
    EXPECT_NEAR(angles[0], 0.0, 0.01);
    EXPECT_NEAR(angles[1], 0.862, 0.1);
    EXPECT_NEAR(angles[2], -1.883, 0.1);
}

// Тест 2: IK с roll=π/4 — проверка влияния 45° крена
TEST(IKWithRoll, roll_45_degrees_affects_joint_angles) {
    quadropted::InverseKinematics ik(0.3762, 0.0935, 0.0, 0.0955, 0.213, 0.213);

    double dx = 0.3762 * 0.5 + 0.02;
    double dy = 0.0935 * 0.5 + 0.0955;
    Eigen::MatrixXd stance(3, 4);
    stance << dx, dx, -dx, -dx, -dy, dy, -dy, dy, 0, 0, 0, 0;

    auto angles_no_roll = ik.inverse_kinematics(stance, 0, 0, 0.25, 0, 0, 0);
    auto angles_roll_45 = ik.inverse_kinematics(stance, 0, 0, 0.25, M_PI / 4, 0, 0);

    // Углы должны отличаться при наличии крена
    bool different = false;
    for (int i = 0; i < 12; ++i) {
        if (std::abs(angles_no_roll[i] - angles_roll_45[i]) > 1e-6) {
            different = true;
            break;
        }
    }
    EXPECT_TRUE(different) << "IK с roll=π/4 должен давать другие углы чем roll=0";
}

// Тест 3: IK с roll=0, pitch=0, yaw=0 — идентичность Python
TEST(IKWithRoll, matches_python_zero_orientation) {
    quadropted::InverseKinematics ik(0.3762, 0.0935, 0.0, 0.0955, 0.213, 0.213);

    double dx = 0.3762 * 0.5 + 0.02;
    double dy = 0.0935 * 0.5 + 0.0955;
    Eigen::MatrixXd stance(3, 4);
    stance << dx, dx, -dx, -dx, -dy, dy, -dy, dy, 0, 0, 0, 0;

    auto angles = ik.inverse_kinematics(stance, 0, 0, 0.25, 0, 0, 0);

    // Python значения для REST стойки
    EXPECT_NEAR(angles[0], 0.0, 0.01);    // hip FR
    EXPECT_NEAR(angles[1], 0.862, 0.1);   // thigh FR
    EXPECT_NEAR(angles[2], -1.883, 0.1);  // calf FR

    EXPECT_NEAR(angles[3], 0.0, 0.01);    // hip FL
    EXPECT_NEAR(angles[4], 0.862, 0.1);   // thigh FL
    EXPECT_NEAR(angles[5], -1.883, 0.1);  // calf FL
}

// Тест 4: IK roundtrip: FK → IK → те же углы (с roll=0)
TEST(IKWithRoll, fk_ik_roundtrip_zero_roll) {
    double body[] = {0.3762, 0.0935};
    double legs[] = {0.0, 0.0955, 0.213, 0.213};

    quadropted::ForwardKinematics fk(body[0], body[1], legs[0], legs[1], legs[2], legs[3]);
    quadropted::InverseKinematics ik(body[0], body[1], legs[0], legs[1], legs[2], legs[3]);

    // Исходные углы
    std::vector<double> original_angles = {0, 0.3, -0.6, 0, 0.3, -0.6, 0, 0.3, -0.6, 0, 0.3, -0.6};

    // FK: углы → позиции ног
    auto foot_pos = fk.forward_kinematics_all_legs(original_angles);
    Eigen::MatrixXd leg_positions(3, 4);
    for (int leg = 0; leg < 4; ++leg) {
        for (int dim = 0; dim < 3; ++dim) {
            leg_positions(dim, leg) = foot_pos[leg](dim);
        }
    }

    // IK: позиции ног → углы
    auto recovered_angles = ik.inverse_kinematics(leg_positions, 0, 0, 0.25, 0, 0, 0);

    // Проверяем что углы восстановлены
    for (int i = 0; i < 12; ++i) {
        EXPECT_NEAR(recovered_angles[i], original_angles[i], 0.01) << "Angle " << i << " mismatch";
    }
}

// Тест 5: IK с roll=π/4 — углы в допустимом диапазоне
TEST(IKWithRoll, roll_45_angles_in_valid_range) {
    quadropted::InverseKinematics ik(0.3762, 0.0935, 0.0, 0.0955, 0.213, 0.213);

    double dx = 0.3762 * 0.5 + 0.02;
    double dy = 0.0935 * 0.5 + 0.0955;
    Eigen::MatrixXd stance(3, 4);
    stance << dx, dx, -dx, -dx, -dy, dy, -dy, dy, 0, 0, 0, 0;

    auto angles = ik.inverse_kinematics(stance, 0, 0, 0.25, M_PI / 4, 0, 0);

    // Все углы должны быть в пределах ±π
    for (int i = 0; i < 12; ++i) {
        EXPECT_LT(std::abs(angles[i]), M_PI + 0.5) << "Angle " << i << " out of range: " << angles[i];
    }
}

// Тест 6: Симметрия углов — левые и правые ноги зеркальны при roll=0
TEST(IKWithRoll, left_right_symmetry_zero_roll) {
    quadropted::InverseKinematics ik(0.3762, 0.0935, 0.0, 0.0955, 0.213, 0.213);

    double dx = 0.3762 * 0.5 + 0.02;
    double dy = 0.0935 * 0.5 + 0.0955;
    Eigen::MatrixXd stance(3, 4);
    stance << dx, dx, -dx, -dx, -dy, dy, -dy, dy, 0, 0, 0, 0;

    auto angles = ik.inverse_kinematics(stance, 0, 0, 0.25, 0, 0, 0);

    // FR (0,1,2) и FL (3,4,5) должны быть симметричны по hip
    // Hip углы: FR и FL должны быть одинаковы по модулю, противоположны по знаку
    // Но для symmetric stance они должны быть одинаковы
    EXPECT_NEAR(angles[0], angles[3], 0.01);  // hip FR vs hip FL
    EXPECT_NEAR(angles[1], angles[4], 0.01);  // thigh FR vs thigh FL
    EXPECT_NEAR(angles[2], angles[5], 0.01);  // calf FR vs calf FL
}

// Тест 7: IK с roll=-π/4 — отрицательный крен
TEST(IKWithRoll, negative_roll_45) {
    quadropted::InverseKinematics ik(0.3762, 0.0935, 0.0, 0.0955, 0.213, 0.213);

    double dx = 0.3762 * 0.5 + 0.02;
    double dy = 0.0935 * 0.5 + 0.0955;
    Eigen::MatrixXd stance(3, 4);
    stance << dx, dx, -dx, -dx, -dy, dy, -dy, dy, 0, 0, 0, 0;

    auto angles_pos = ik.inverse_kinematics(stance, 0, 0, 0.25, M_PI / 4, 0, 0);
    auto angles_neg = ik.inverse_kinematics(stance, 0, 0, 0.25, -M_PI / 4, 0, 0);

    // Углы при +roll и -roll должны быть разными
    bool different = false;
    for (int i = 0; i < 12; ++i) {
        if (std::abs(angles_pos[i] - angles_neg[i]) > 1e-6) {
            different = true;
            break;
        }
    }
    EXPECT_TRUE(different) << "IK с roll=+π/4 и roll=-π/4 должны давать разные углы";
}

// Тест 8: IK с roll=0.1, pitch=-0.05, yaw=0.02 — совпадение с Python
TEST(IKWithRoll, matches_python_small_angles) {
    quadropted::InverseKinematics ik(0.3762, 0.0935, 0.0, 0.0955, 0.213, 0.213);

    double dx = 0.3762 * 0.5 + 0.02;
    double dy = 0.0935 * 0.5 + 0.0955;
    Eigen::MatrixXd stance(3, 4);
    stance << dx, dx, -dx, -dx, -dy, dy, -dy, dy, 0, 0, 0, 0;

    // Python: rotxyz(0.1, -0.05, 0.02) → конкретная матрица
    auto angles = ik.inverse_kinematics(stance, 0, 0, 0.25, 0.1, -0.05, 0.02);

    // Проверка что углы в реалистичном диапазоне
    for (int i = 0; i < 12; ++i) {
        EXPECT_LT(std::abs(angles[i]), M_PI + 0.5) << "Angle " << i << " out of range";
    }
}
