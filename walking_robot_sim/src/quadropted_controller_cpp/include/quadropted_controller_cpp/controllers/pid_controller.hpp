#pragma once
#include <array>

namespace quadropted {

class PIDController {
  public:
    PIDController(double kp, double ki, double kd);

    std::array<double, 2> run(double roll, double pitch, double current_time);
    void reset(double current_time);
    void set_desired(double roll, double pitch);

    const std::array<double, 2>& last_error() const { return last_error_; }
    const std::array<double, 2>& i_term() const { return i_term_; }
    const std::array<double, 2>& d_term() const { return d_term_; }

  private:
    double kp_, ki_, kd_;
    std::array<double, 2> desired_roll_pitch_{0.0, 0.0};
    std::array<double, 2> i_term_{0.0, 0.0}, d_term_{0.0, 0.0};
    std::array<double, 2> last_error_{0.0, 0.0};
    double max_i_ = 0.2;
    double last_time_ = -1.0;
};

}  // namespace quadropted
