#include "quadropted_controller_cpp/controllers/gait_controller.hpp"

namespace quadropted {

GaitController::GaitController(double stance_time, double swing_time, double time_step, Eigen::MatrixXi contact_phases,
                               Eigen::MatrixXd default_stance)
    : stance_time_(stance_time),
      swing_time_(swing_time),
      time_step_(time_step),
      contact_phases_(std::move(contact_phases)),
      default_stance_(std::move(default_stance)) {
    stance_ticks_ = static_cast<int>(stance_time_ / time_step_);
    swing_ticks_ = static_cast<int>(swing_time_ / time_step_);
    compute_phase_ticks();
    phase_length_ = 0;
    for (int t : phase_ticks_)
        phase_length_ += t;
}

void GaitController::compute_phase_ticks() {
    phase_ticks_.clear();
    int num_phases = contact_phases_.cols();
    for (int i = 0; i < num_phases; ++i) {
        bool has_swing = false;
        for (int leg = 0; leg < contact_phases_.rows(); ++leg) {
            if (contact_phases_(leg, i) == 0) {
                has_swing = true;
                break;
            }
        }
        if (has_swing) {
            phase_ticks_.push_back(swing_ticks_);
        } else {
            phase_ticks_.push_back(stance_ticks_);
        }
    }
}

int GaitController::phase_index(int ticks) const {
    int phase_time = ticks % phase_length_;
    int phase_sum = 0;
    int num_phases = static_cast<int>(phase_ticks_.size());
    for (int i = 0; i < num_phases; ++i) {
        phase_sum += phase_ticks_[i];
        if (phase_time < phase_sum) {
            return i;
        }
    }
    return num_phases - 1;
}

int GaitController::subphase_ticks(int ticks) const {
    int phase_time = ticks % phase_length_;
    int phase_sum = 0;
    int num_phases = static_cast<int>(phase_ticks_.size());
    for (int i = 0; i < num_phases; ++i) {
        phase_sum += phase_ticks_[i];
        if (phase_time < phase_sum) {
            return phase_time - phase_sum + phase_ticks_[i];
        }
    }
    return 0;
}

Eigen::VectorXi GaitController::contacts(int ticks) const {
    return contact_phases_.col(phase_index(ticks));
}

}  // namespace quadropted
