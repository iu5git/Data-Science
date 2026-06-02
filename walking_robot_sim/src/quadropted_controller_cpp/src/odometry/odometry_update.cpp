#include <cmath>

#include "quadropted_controller_cpp/odometry/odometry.hpp"

namespace quadropted {

double normalize_angle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

void update_odometry(OdometryState& state, double dt, double contact_count_coeff) {
    if (dt <= 0.0) return;

    double delta_x_total = 0.0;
    double delta_y_total = 0.0;
    double contact_sum = 0.0;
    int actual_contacts = 0;

    for (int i = 0; i < 4; ++i) {
        if (state.foot_contacts[i]) {
            double foot_rel_x = state.foot_positions[i].x();
            double foot_rel_y = state.foot_positions[i].y();

            if (state.prev_foot_positions[i].has_value()) {
                double delta_x = foot_rel_x - state.prev_foot_positions[i]->x();
                double delta_y = foot_rel_y - state.prev_foot_positions[i]->y();

                delta_x_total += delta_x;
                delta_y_total += -delta_y;
                contact_sum += contact_count_coeff;
                actual_contacts++;
            }

            state.prev_foot_positions[i] = Eigen::Vector2d(foot_rel_x, foot_rel_y);
        }
    }

    double avg_delta_x, avg_delta_y;

    if (contact_sum > 0) {
        avg_delta_x = delta_x_total / contact_sum;
        avg_delta_y = delta_y_total / contact_sum;
    } else {
        avg_delta_x = state.linear_velocity_x * dt;
        avg_delta_y = state.linear_velocity_y * dt;
    }

    state.append_delta(avg_delta_x, avg_delta_y);
    auto avg = state.average_delta();

    double cos_theta = std::cos(state.theta);
    double sin_theta = std::sin(state.theta);
    state.x += avg.first * cos_theta - avg.second * sin_theta;
    state.y += avg.first * sin_theta + avg.second * cos_theta;
}

}  // namespace quadropted
