#pragma once
#include <Eigen/Dense>
#include <array>

namespace quadropted {

enum class BehaviorState { REST = 0, TROT, CRAWL, STAND };

struct State {
    double body_height = 0.25;
    Eigen::MatrixXd foot_locations;  // (3, 4)
    std::array<double, 3> body_local_position{0, 0, 0};
    std::array<double, 3> body_local_orientation{0, 0, 0};
    double imu_roll = 0, imu_pitch = 0;
    int ticks = 0;
    BehaviorState behavior_state = BehaviorState::REST;
    double robot_height = -0.25;  // FIX: отрицательная как в Python StateCommand.py

    State() : foot_locations(Eigen::MatrixXd::Zero(3, 4)) {}
    explicit State(double height)
        : body_height(height), robot_height(-height), foot_locations(Eigen::MatrixXd::Zero(3, 4)) {}
};

struct Command {
    std::array<double, 3> velocity{0, 0, 0};
    std::array<double, 3> yaw_rate{0, 0, 0};
    double robot_height = -0.25;  // FIX: отрицательная как в Python StateCommand.py
    bool trot_event = true, rest_event = true, crawl_event = false, stand_event = false;
};

}  // namespace quadropted
