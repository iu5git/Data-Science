#include "quadropted_controller_cpp/controllers/rest_controller.hpp"

#include <cmath>

#include "quadropted_controller_cpp/utils/rotation_matrices.hpp"

namespace quadropted {

RestController::RestController(Eigen::MatrixXd default_stance)
    : default_stance_(std::move(default_stance)), pid_(0.75, 2.29, 0.0), use_imu_(false), pid_last_time_(0.0) {}

void RestController::reset() {
    pid_.reset(0.0);
    pid_last_time_ = 0.0;
}

Eigen::MatrixXd RestController::step(const State& state, const Command& cmd) {
    Eigen::MatrixXd temp = default_stance_;
    temp.row(2).setConstant(cmd.robot_height);

    if (use_imu_) {
        // Timestamp не критичен для REST — PID используется только для integral
        auto compensation = pid_.run(state.imu_roll, state.imu_pitch, pid_last_time_);
        pid_last_time_ += 0.02;  // фиксированный шаг как в Python
        double roll_compensation = -compensation[0];
        double pitch_compensation = -compensation[1];
        Eigen::Matrix3d rot = rotxyz(roll_compensation, pitch_compensation, 0.0);
        temp = rot * temp;
    }

    return temp;
}

}  // namespace quadropted
