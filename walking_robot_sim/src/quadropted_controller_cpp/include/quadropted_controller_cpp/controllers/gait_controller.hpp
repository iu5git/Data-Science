#pragma once
#include <Eigen/Dense>
#include <vector>

namespace quadropted {

class GaitController {
  public:
    GaitController(double stance_time, double swing_time, double time_step, Eigen::MatrixXi contact_phases,
                   Eigen::MatrixXd default_stance);

    const Eigen::MatrixXd& default_stance() const { return default_stance_; }
    int stance_ticks() const { return stance_ticks_; }
    int swing_ticks() const { return swing_ticks_; }
    int phase_length() const { return phase_length_; }
    const std::vector<int>& phase_ticks() const { return phase_ticks_; }

    int phase_index(int ticks) const;
    int subphase_ticks(int ticks) const;
    Eigen::VectorXi contacts(int ticks) const;

  protected:
    double stance_time_, swing_time_, time_step_;
    Eigen::MatrixXi contact_phases_;
    Eigen::MatrixXd default_stance_;
    int stance_ticks_ = 0, swing_ticks_ = 0, phase_length_ = 0;
    std::vector<int> phase_ticks_;
    void compute_phase_ticks();
};

}  // namespace quadropted
