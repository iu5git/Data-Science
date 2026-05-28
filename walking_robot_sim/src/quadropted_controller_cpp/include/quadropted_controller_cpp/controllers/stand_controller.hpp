#pragma once
#include <Eigen/Dense>

#include "quadropted_controller_cpp/controllers/pid_controller.hpp"
#include "quadropted_controller_cpp/states/state_command.hpp"

namespace quadropted {

class StandController {
  public:
    explicit StandController(Eigen::MatrixXd default_stance);
    Eigen::MatrixXd run(State& state, Command& cmd) const;
    const Eigen::MatrixXd& default_stance() const { return default_stance_; }

  private:
    Eigen::MatrixXd default_stance_;
    double body_velocity_scale_ = 0.01;
    double body_angular_scale_ = 0.005;
    // Увеличено с 0.035 → 0.2 чтобы teleop speed мог влиять на скорость
    // teleop speed=1.0 → velocity=0.035 (cmd_vel_pub linear_scale) → после clamp=0.035
    // teleop speed=5.0 → velocity=0.175 → после clamp=0.175 (быстрее в 5×)
    double max_linear_velocity_ = 0.2;
    double max_angular_velocity_ = 0.5;
};

}  // namespace quadropted
