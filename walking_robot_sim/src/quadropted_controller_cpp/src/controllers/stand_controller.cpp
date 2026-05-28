#include "quadropted_controller_cpp/controllers/stand_controller.hpp"

#include <algorithm>
#include <cmath>

namespace quadropted {

StandController::StandController(Eigen::MatrixXd default_stance) : default_stance_(std::move(default_stance)) {}

Eigen::MatrixXd StandController::run(State& state, Command& cmd) const {
    Eigen::MatrixXd temp = default_stance_;
    temp.row(2).setConstant(cmd.robot_height);

    Eigen::Vector3d linear_vel(cmd.velocity[0], cmd.velocity[1], cmd.velocity[2]);
    Eigen::Vector3d angular_vel(cmd.yaw_rate[0], cmd.yaw_rate[1], cmd.yaw_rate[2]);

    // Clamp velocities
    linear_vel = linear_vel.cwiseMax(-max_linear_velocity_).cwiseMin(max_linear_velocity_);
    angular_vel = angular_vel.cwiseMax(-max_angular_velocity_).cwiseMin(max_angular_velocity_);

    // Проверка на stop (все скорости близки к нулю)
    bool has_command = std::abs(linear_vel.x()) > 1e-4 || std::abs(linear_vel.y()) > 1e-4 ||
                       std::abs(linear_vel.z()) > 1e-4 || std::abs(angular_vel.x()) > 1e-4 ||
                       std::abs(angular_vel.y()) > 1e-4 || std::abs(angular_vel.z()) > 1e-4;

    if (has_command) {
        // Активное управление — накапливаем позицию/ориентацию
        state.body_local_position[0] += linear_vel.x() * body_velocity_scale_;
        state.body_local_position[1] += linear_vel.y() * body_velocity_scale_;
        state.body_local_position[2] += linear_vel.z() * body_velocity_scale_;

        state.body_local_orientation[0] += angular_vel.x() * body_angular_scale_;
        state.body_local_orientation[1] += angular_vel.y() * body_angular_scale_;
        state.body_local_orientation[2] += angular_vel.z() * body_angular_scale_;
    } else {
        // Stop (пробел) — плавный возврат к центру (как lerp в TROT/CRAWL)
        constexpr double alpha_pos = 0.05;  // ~20 тиков (0.33с) для возврата
        constexpr double alpha_ori = 0.05;

        state.body_local_position[0] *= (1.0 - alpha_pos);
        state.body_local_position[1] *= (1.0 - alpha_pos);
        state.body_local_position[2] *= (1.0 - alpha_pos);

        state.body_local_orientation[0] *= (1.0 - alpha_ori);
        state.body_local_orientation[1] *= (1.0 - alpha_ori);
        state.body_local_orientation[2] *= (1.0 - alpha_ori);
    }

    return temp;
}

}  // namespace quadropted
