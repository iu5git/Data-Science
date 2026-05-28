#include "quadropted_controller_cpp/controllers/pid_controller.hpp"

#include <cmath>

namespace quadropted {

PIDController::PIDController(double kp, double ki, double kd) : kp_(kp), ki_(ki), kd_(kd) {}

std::array<double, 2> PIDController::run(double roll, double pitch, double current_time) {
    std::array<double, 2> error = {desired_roll_pitch_[0] - roll, desired_roll_pitch_[1] - pitch};

    if (last_time_ < 0.0) {
        last_time_ = current_time;
        return {0.0, 0.0};
    }

    double step = current_time - last_time_;
    if (step < 1e-6) {
        return {0.0, 0.0};
    }

    for (int i = 0; i < 2; ++i) {
        i_term_[i] += error[i] * step;
        if (i_term_[i] < -max_i_)
            i_term_[i] = -max_i_;
        else if (i_term_[i] > max_i_)
            i_term_[i] = max_i_;

        d_term_[i] = (error[i] - last_error_[i]) / step;
    }

    last_time_ = current_time;
    last_error_ = error;

    std::array<double, 2> result;
    for (int i = 0; i < 2; ++i) {
        result[i] = kp_ * error[i] + ki_ * i_term_[i] + kd_ * d_term_[i];
    }

    return result;
}

void PIDController::reset(double current_time) {
    last_time_ = current_time;
    i_term_ = {0.0, 0.0};
    d_term_ = {0.0, 0.0};
    last_error_ = {0.0, 0.0};
}

void PIDController::set_desired(double roll, double pitch) {
    desired_roll_pitch_[0] = roll;
    desired_roll_pitch_[1] = pitch;
}

}  // namespace quadropted
