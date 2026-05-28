#pragma once
#include <Eigen/Dense>

#include "quadropted_controller_cpp/controllers/pid_controller.hpp"
#include "quadropted_controller_cpp/states/state_command.hpp"

namespace quadropted {

class RestController {
  public:
    explicit RestController(Eigen::MatrixXd default_stance);
    Eigen::MatrixXd step(const State& state, const Command& cmd);
    const Eigen::MatrixXd& default_stance() const { return default_stance_; }
    PIDController& pid() { return pid_; }
    bool use_imu() const { return use_imu_; }
    void set_use_imu(bool v) { use_imu_ = v; }
    void reset();

  private:
    Eigen::MatrixXd default_stance_;
    PIDController pid_;
    mutable bool use_imu_;
    mutable double pid_last_time_ = 0.0;
};

}  // namespace quadropted
