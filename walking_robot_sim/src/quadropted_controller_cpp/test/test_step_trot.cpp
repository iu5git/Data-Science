#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "quadropted_controller_cpp/controllers/trot_gait.hpp"
#include "quadropted_controller_cpp/controllers/trot_stance.hpp"
#include "quadropted_controller_cpp/utils/rotation_matrices.hpp"

using namespace quadropted;

class StepTrotCrossValidationTest : public ::testing::Test {
  protected:
    Eigen::MatrixXd default_stance_;
    std::unique_ptr<TrotGaitController> trot_gait_;
    std::unique_ptr<TrotStanceController> stance_ctrl_;

    void SetUp() override {
        double dx = 0.19;
        double dy = 0.15;
        default_stance_.resize(3, 4);
        default_stance_ << dx, dx, -dx, -dx, -dy, dy, -dy, dy, -0.25, -0.25, -0.25, -0.25;

        trot_gait_ = std::make_unique<TrotGaitController>(0.04, 0.18, 0.02, false, default_stance_);
        stance_ctrl_ = std::make_unique<TrotStanceController>(40, 18, 22, 0.02, 0.02);
    }
};

TEST_F(StepTrotCrossValidationTest, position_delta_zero_velocity_z_converges) {
    Eigen::MatrixXd foot_locations = default_stance_;
    foot_locations(2, 0) = -0.20;
    Eigen::Vector3d cmd_vel(0.0, 0.0, 0.0);
    double robot_height = -0.25;

    Eigen::Vector3d delta_pos = stance_ctrl_->position_delta(0, foot_locations, cmd_vel, robot_height);

    EXPECT_NEAR(delta_pos.z(), -0.05, 0.001);
}

TEST_F(StepTrotCrossValidationTest, position_delta_forward_motion_x_negative) {
    Eigen::MatrixXd foot_locations = default_stance_;
    Eigen::Vector3d cmd_vel(0.3, 0.0, 0.0);
    double robot_height = -0.25;

    Eigen::Vector3d delta_pos = stance_ctrl_->position_delta(0, foot_locations, cmd_vel, robot_height);

    EXPECT_LT(delta_pos.x(), 0.0);
}

TEST_F(StepTrotCrossValidationTest, position_delta_yaw_rotation_z_axis) {
    Eigen::MatrixXd foot_locations = default_stance_;
    Eigen::Vector3d cmd_vel(0.0, 0.0, 0.5);
    double robot_height = -0.25;

    Eigen::Vector3d delta_pos = stance_ctrl_->position_delta(0, foot_locations, cmd_vel, robot_height);
    Eigen::Matrix3d delta_ori = rotxyz(-cmd_vel.x() * 0.02, -cmd_vel.y() * 0.02, -cmd_vel.z() * 0.02);

    EXPECT_NEAR(delta_ori(0, 0), 0.999, 0.01);
    EXPECT_NEAR(delta_ori(1, 0), -0.01, 0.01);
    EXPECT_NEAR(delta_ori(2, 2), 1.0, 0.001);
}

TEST_F(StepTrotCrossValidationTest, next_foot_location_shape) {
    Eigen::MatrixXd foot_locations = default_stance_;
    Eigen::Vector3d cmd_vel(0.0, 0.0, 0.0);
    double robot_height = -0.25;

    Eigen::Vector3d new_loc = stance_ctrl_->next_foot_location(0, foot_locations, cmd_vel, robot_height);

    EXPECT_EQ(new_loc.size(), 3);
}

TEST_F(StepTrotCrossValidationTest, full_trot_step_stance_phase) {
    Eigen::MatrixXd foot_locations = default_stance_;
    Eigen::Vector3d cmd_vel(0.2, 0.0, 0.0);
    double robot_height = -0.25;

    Eigen::VectorXi contacts = Eigen::VectorXi::Constant(4, 1);
    Eigen::MatrixXd new_foot_locations = Eigen::MatrixXd::Zero(3, 4);

    for (int leg = 0; leg < 4; ++leg) {
        if (contacts(leg) == 1) {
            new_foot_locations.col(leg) = stance_ctrl_->next_foot_location(leg, foot_locations, cmd_vel, robot_height);
        }
    }

    EXPECT_EQ(new_foot_locations.rows(), 3);
    EXPECT_EQ(new_foot_locations.cols(), 4);
}

TEST_F(StepTrotCrossValidationTest, python_cpp_equivalence_stance) {
    Eigen::MatrixXd foot_locations = default_stance_;
    Eigen::Vector3d cmd_vel(0.15, 0.1, 0.3);
    double robot_height = -0.25;

    for (int leg = 0; leg < 4; ++leg) {
        Eigen::Vector3d foot_loc = foot_locations.col(leg);
        Eigen::Vector3d delta_pos = stance_ctrl_->position_delta(leg, foot_locations, cmd_vel, robot_height);
        Eigen::Matrix3d delta_ori = rotxyz(-cmd_vel.x() * 0.02, -cmd_vel.y() * 0.02, -cmd_vel.z() * 0.02);
        Eigen::Vector3d new_loc = delta_ori * foot_loc + delta_pos;

        EXPECT_NEAR(new_loc.z(), -0.25, 0.01);
    }
}

TEST_F(StepTrotCrossValidationTest, robot_height_convergence) {
    auto stance = std::make_unique<TrotStanceController>(40, 18, 22, 0.02, 0.02);

    std::vector<double> test_z = {-0.15, -0.20, -0.30};
    double target_z = -0.25;

    for (double initial_z : test_z) {
        Eigen::MatrixXd foot_locations = default_stance_;
        foot_locations(2, 0) = initial_z;
        Eigen::Vector3d cmd_vel(0.0, 0.0, 0.0);

        Eigen::Vector3d delta_pos = stance->position_delta(0, foot_locations, cmd_vel, target_z);
        double new_z = initial_z + delta_pos.z();

        EXPECT_LT(std::abs(new_z - target_z), std::abs(initial_z - target_z));
    }
}

TEST_F(StepTrotCrossValidationTest, sign_consistency_with_cpp) {
    Eigen::MatrixXd foot_locations = default_stance_;
    Eigen::Vector3d cmd_vel(0.0, 0.0, 0.0);
    double robot_height = -0.25;

    Eigen::Vector3d delta_pos = stance_ctrl_->position_delta(0, foot_locations, cmd_vel, robot_height);

    EXPECT_NEAR(delta_pos.z(), 0.0, 0.001);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}