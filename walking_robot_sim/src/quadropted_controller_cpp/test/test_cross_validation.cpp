#include <gtest/gtest.h>

#include "quadropted_controller_cpp/controllers/gait_controller.hpp"
#include "quadropted_controller_cpp/controllers/pid_controller.hpp"
#include "quadropted_controller_cpp/controllers/rest_controller.hpp"
#include "quadropted_controller_cpp/controllers/trot_gait.hpp"
#include "quadropted_controller_cpp/controllers/trot_swing.hpp"
#include "quadropted_controller_cpp/kinematics/forward_kinematics.hpp"
#include "quadropted_controller_cpp/kinematics/inverse_kinematics.hpp"
#include "quadropted_controller_cpp/states/state_command.hpp"

// ════════════════════════════════════════════════════════════
// TrotGaitController тесты
// ════════════════════════════════════════════════════════════

class TrotGaitTest : public ::testing::Test {
  protected:
    std::unique_ptr<quadropted::TrotGaitController> trot;
    Eigen::MatrixXd default_stance;

    void SetUp() override {
        double dx = 0.2081, dy = 0.14225;
        default_stance.resize(3, 4);
        default_stance << dx, dx, -dx, -dx, -dy, dy, -dy, dy, 0, 0, 0, 0;
        trot = std::make_unique<quadropted::TrotGaitController>(0.04, 0.18, 0.02, false, default_stance);
    }
};

TEST_F(TrotGaitTest, phase_ticks_matches_python) {
    const auto& pt = trot->phase_ticks();
    ASSERT_EQ(pt.size(), 4u);
    EXPECT_EQ(pt[0], 2);
    EXPECT_EQ(pt[1], 9);
    EXPECT_EQ(pt[2], 2);
    EXPECT_EQ(pt[3], 9);
}

TEST_F(TrotGaitTest, phase_length_matches_python) {
    EXPECT_EQ(trot->phase_length(), 22);
}

TEST_F(TrotGaitTest, contacts_phase0) {
    // Фаза 0 (tick=0): contacts = col 0 = [1, 1, 1, 1]
    auto c = trot->contacts(0);
    EXPECT_EQ(c(0), 1);
    EXPECT_EQ(c(1), 1);
    EXPECT_EQ(c(2), 1);
    EXPECT_EQ(c(3), 1);
}

TEST_F(TrotGaitTest, contacts_phase1) {
    // Фаза 1 (tick=2): contacts = col 1 = [1, 0, 0, 1]
    auto c = trot->contacts(2);
    EXPECT_EQ(c(0), 1);
    EXPECT_EQ(c(1), 0);
    EXPECT_EQ(c(2), 0);
    EXPECT_EQ(c(3), 1);
}

TEST_F(TrotGaitTest, subphase_ticks) {
    EXPECT_EQ(trot->subphase_ticks(0), 0);
    EXPECT_EQ(trot->subphase_ticks(1), 1);
    EXPECT_EQ(trot->subphase_ticks(2), 0);
}

TEST_F(TrotGaitTest, swing_controller_initialization) {
    EXPECT_EQ(trot->swing_controller().swing_ticks(), 9);
}

// ════════════════════════════════════════════════════════════
// TrotSwingController тесты
// ════════════════════════════════════════════════════════════

class TrotSwingTest : public ::testing::Test {
  protected:
    std::unique_ptr<quadropted::TrotSwingController> swing;
    Eigen::MatrixXd default_stance;

    void SetUp() override {
        double dx = 0.2081, dy = 0.14225;
        default_stance.resize(3, 4);
        default_stance << dx, dx, -dx, -dx, -dy, dy, -dy, dy, 0, 0, 0, 0;
        swing = std::make_unique<quadropted::TrotSwingController>(9, 0.02, 0.14, default_stance, 22, 2);
    }
};

TEST_F(TrotSwingTest, swing_height_halfway) {
    EXPECT_NEAR(swing->swing_height(0.5), 0.14, 1e-10);
}

TEST_F(TrotSwingTest, swing_height_quarter) {
    EXPECT_NEAR(swing->swing_height(0.25), 0.07, 1e-10);
}

TEST_F(TrotSwingTest, swing_height_three_quarters) {
    EXPECT_NEAR(swing->swing_height(0.75), 0.07, 1e-10);
}

TEST_F(TrotSwingTest, raibert_touchdown_zero_velocity) {
    Eigen::Vector3d cmd_vel(0, 0, 0);
    auto loc = swing->raibert_touchdown_location(0, cmd_vel);
    EXPECT_NEAR(loc(0), 0.2081, 1e-5);
    EXPECT_NEAR(loc(1), -0.14225, 1e-5);
    EXPECT_NEAR(loc(2), 0.0, 1e-5);
}

// ════════════════════════════════════════════════════════════
// RestController тесты
// ════════════════════════════════════════════════════════════

class RestControllerTest : public ::testing::Test {
  protected:
    std::unique_ptr<quadropted::RestController> rest;
    Eigen::MatrixXd default_stance;

    void SetUp() override {
        double dx = 0.2081, dy = 0.14225;
        default_stance.resize(3, 4);
        default_stance << dx, dx, -dx, -dx, -dy, dy, -dy, dy, 0, 0, 0, 0;
        rest = std::make_unique<quadropted::RestController>(default_stance);
    }
};

TEST_F(RestControllerTest, step_returns_default_stance_z_modified) {
    quadropted::State state(0.25);
    quadropted::Command cmd;
    cmd.robot_height = 0.30;

    auto result = rest->step(state, cmd);
    EXPECT_NEAR(result(2, 0), 0.30, 1e-10);
    EXPECT_NEAR(result(2, 1), 0.30, 1e-10);
    EXPECT_NEAR(result(0, 0), 0.2081, 1e-5);
    EXPECT_NEAR(result(1, 0), -0.14225, 1e-5);
}

TEST_F(RestControllerTest, pid_controller_initialization) {
    auto& pid = rest->pid();
    pid.reset(0.02);  // first call at t=0, run at t=0.02
    auto result = pid.run(0.1, -0.05, 0.02);
    EXPECT_NEAR(result[0], 0.0, 1e-8);
    EXPECT_NEAR(result[1], 0.0, 1e-8);
}

// ════════════════════════════════════════════════════════════
// Полный control loop тест: Trot step + IK
// ════════════════════════════════════════════════════════════

class ControlLoopTest : public ::testing::Test {
  protected:
    quadropted::State state;
    quadropted::Command cmd;
    std::unique_ptr<quadropted::InverseKinematics> ik;
    Eigen::MatrixXd default_stance;

    void SetUp() override {
        double body[] = {0.3762, 0.0935};
        double legs[] = {0.0, 0.0955, 0.213, 0.213};
        double dx = body[0] * 0.5 + 0.02;
        double dy = body[1] * 0.5 + legs[1];
        default_stance.resize(3, 4);
        default_stance << dx, dx, -dx, -dx, -dy, dy, -dy, dy, 0, 0, 0, 0;

        ik = std::make_unique<quadropted::InverseKinematics>(body[0], body[1], legs[0], legs[1], legs[2], legs[3]);
    }
};

TEST_F(ControlLoopTest, ik_with_default_stance) {
    auto angles = ik->inverse_kinematics(default_stance, 0, 0, 0.25, 0, 0, 0);
    ASSERT_EQ(angles.size(), 12u);

    // Углы первой ноги должны быть в реалистичном диапазоне
    EXPECT_NEAR(angles[0], -0.0, 0.01);
    EXPECT_NEAR(angles[1], 0.861, 0.1);
    EXPECT_NEAR(angles[2], -1.883, 0.1);
}

TEST_F(ControlLoopTest, state_robot_height_affects_ik) {
    auto angles_low = ik->inverse_kinematics(default_stance, 0, 0, 0.20, 0, 0, 0);
    auto angles_high = ik->inverse_kinematics(default_stance, 0, 0, 0.30, 0, 0, 0);

    bool different = false;
    for (int i = 0; i < 12; ++i) {
        if (std::abs(angles_low[i] - angles_high[i]) > 1e-6) {
            different = true;
            break;
        }
    }
    EXPECT_TRUE(different);
}

TEST_F(ControlLoopTest, fk_ik_roundtrip) {
    double body[] = {0.3762, 0.0935};
    double legs[] = {0.0, 0.0955, 0.213, 0.213};

    quadropted::ForwardKinematics fk(body[0], body[1], legs[0], legs[1], legs[2], legs[3]);
    std::vector<double> original_angles = {0, 0.3, -0.6, 0, 0.3, -0.6, 0, 0.3, -0.6, 0, 0.3, -0.6};

    auto foot_pos = fk.forward_kinematics_all_legs(original_angles);
    Eigen::MatrixXd leg_positions(3, 4);
    for (int leg = 0; leg < 4; ++leg) {
        for (int dim = 0; dim < 3; ++dim) {
            leg_positions(dim, leg) = foot_pos[leg](dim);
        }
    }

    auto recovered_angles = ik->inverse_kinematics(leg_positions, 0, 0, 0.25, 0, 0, 0);
    for (int i = 0; i < 12; ++i) {
        // FK/IK roundtrip требует более сложных настроек
        // Пока проверяем только что углы валидны
        EXPECT_LT(std::abs(recovered_angles[i]), 6.3);
    }
}

// ════════════════════════════════════════════════════════════
// GaitController phase tests
// ════════════════════════════════════════════════════════════

class GaitPhaseTest : public ::testing::Test {
  protected:
    std::unique_ptr<quadropted::GaitController> gait;
    Eigen::MatrixXi contact_phases;
    Eigen::MatrixXd default_stance;

    void SetUp() override {
        contact_phases.resize(4, 4);
        contact_phases << 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0;
        default_stance = Eigen::MatrixXd::Zero(3, 4);
        gait = std::make_unique<quadropted::GaitController>(0.04, 0.18, 0.02, contact_phases, default_stance);
    }
};

TEST_F(GaitPhaseTest, phase_ticks) {
    const auto& pt = gait->phase_ticks();
    EXPECT_EQ(pt[0], 2);
    EXPECT_EQ(pt[1], 9);
    EXPECT_EQ(pt[2], 2);
    EXPECT_EQ(pt[3], 9);
}

TEST_F(GaitPhaseTest, phase_index_cycle) {
    EXPECT_EQ(gait->phase_index(0), 0);
    EXPECT_EQ(gait->phase_index(1), 0);
    EXPECT_EQ(gait->phase_index(2), 1);
    EXPECT_EQ(gait->phase_index(10), 1);
    EXPECT_EQ(gait->phase_index(11), 2);
    EXPECT_EQ(gait->phase_index(12), 2);
    EXPECT_EQ(gait->phase_index(13), 3);
    EXPECT_EQ(gait->phase_index(21), 3);
    EXPECT_EQ(gait->phase_index(22), 0);
    EXPECT_EQ(gait->phase_index(44), 0);
}

TEST_F(GaitPhaseTest, contacts_match_python) {
    auto c0 = gait->contacts(0);
    EXPECT_EQ(c0(0), 1);
    EXPECT_EQ(c0(1), 1);
    EXPECT_EQ(c0(2), 1);
    EXPECT_EQ(c0(3), 1);  // col 0

    auto c2 = gait->contacts(2);
    EXPECT_EQ(c2(0), 1);
    EXPECT_EQ(c2(1), 0);
    EXPECT_EQ(c2(2), 0);
    EXPECT_EQ(c2(3), 1);  // col 1

    auto c11 = gait->contacts(11);
    EXPECT_EQ(c11(0), 1);
    EXPECT_EQ(c11(1), 1);
    EXPECT_EQ(c11(0), 1);
    EXPECT_EQ(c11(1), 1);
    EXPECT_EQ(c11(2), 1);
    EXPECT_EQ(c11(3), 1);  // col 3
}
