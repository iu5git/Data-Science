#pragma once
#include <Eigen/Dense>
#include <array>
#include <deque>
#include <optional>

namespace quadropted {

struct OdometryState {
    double x = 0.0, y = 0.0, theta = 0.0;
    double linear_velocity_x = 0.0, linear_velocity_y = 0.0, imu_angular_velocity = 0.0;

    int filter_window_size = 14;
    std::deque<double> delta_x_queue, delta_y_queue;
    double sum_delta_x = 0.0, sum_delta_y = 0.0;

    std::array<Eigen::Vector3d, 4> foot_positions{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                                  Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    std::array<std::optional<Eigen::Vector2d>, 4> prev_foot_positions{};
    std::array<bool, 4> foot_contacts{false, false, false, false};
    std::array<double, 12> joint_positions{};

    int gazebo_clock_sec = 0, gazebo_clock_nanosec = 0, encoder_pos = 0;

    OdometryState() = default;
    explicit OdometryState(int window);

    void append_delta(double dx, double dy);
    std::pair<double, double> average_delta() const;
    void reset();
};

double normalize_angle(double angle);
void update_odometry(OdometryState& state, double dt, double contact_count_coeff = 0.65);

}  // namespace quadropted
