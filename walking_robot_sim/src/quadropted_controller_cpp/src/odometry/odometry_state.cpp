#include <algorithm>

#include "quadropted_controller_cpp/odometry/odometry.hpp"

namespace quadropted {

OdometryState::OdometryState(int window)
    : filter_window_size(window), delta_x_queue(), delta_y_queue(), sum_delta_x(0.0), sum_delta_y(0.0) {
    // Инициализируем optional массивы
    for (int i = 0; i < 4; ++i) {
        prev_foot_positions[i] = std::nullopt;
    }
}

void OdometryState::append_delta(double dx, double dy) {
    if (static_cast<int>(delta_x_queue.size()) == filter_window_size) {
        sum_delta_x -= delta_x_queue.front();
        sum_delta_y -= delta_y_queue.front();
        delta_x_queue.pop_front();
        delta_y_queue.pop_front();
    }
    delta_x_queue.push_back(dx);
    delta_y_queue.push_back(dy);
    sum_delta_x += dx;
    sum_delta_y += dy;
}

std::pair<double, double> OdometryState::average_delta() const {
    int n = static_cast<int>(delta_x_queue.size());
    if (n == 0) return {0.0, 0.0};
    return {sum_delta_x / n, sum_delta_y / n};
}

void OdometryState::reset() {
    x = 0.0;
    y = 0.0;
    theta = 0.0;
    linear_velocity_x = 0.0;
    linear_velocity_y = 0.0;
    imu_angular_velocity = 0.0;
    delta_x_queue.clear();
    delta_y_queue.clear();
    sum_delta_x = 0.0;
    sum_delta_y = 0.0;
    for (int i = 0; i < 4; ++i) {
        foot_positions[i] = Eigen::Vector3d::Zero();
        prev_foot_positions[i] = std::nullopt;
        foot_contacts[i] = false;
    }
    joint_positions.fill(0.0);
    gazebo_clock_sec = 0;
    gazebo_clock_nanosec = 0;
    encoder_pos = 0;
}

}  // namespace quadropted
