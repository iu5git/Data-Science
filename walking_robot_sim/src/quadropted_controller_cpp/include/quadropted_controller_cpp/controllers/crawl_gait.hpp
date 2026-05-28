#pragma once
#include "quadropted_controller_cpp/controllers/crawl_stance.hpp"
#include "quadropted_controller_cpp/controllers/crawl_swing.hpp"
#include "quadropted_controller_cpp/controllers/gait_controller.hpp"

namespace quadropted {

class CrawlGaitController : public GaitController {
  public:
    CrawlGaitController(double stance_time, double swing_time, double time_step, Eigen::MatrixXd default_stance);
    Eigen::MatrixXd step(int ticks, const Eigen::MatrixXd& current, const Eigen::Vector3d& cmd_vel) const;
    void reset();
    CrawlSwingController& swing() { return swing_; }
    CrawlStanceController& stance() { return stance_; }
    bool is_first_cycle() const { return first_cycle_; }

  private:
    CrawlSwingController swing_;
    CrawlStanceController stance_;
    mutable bool first_cycle_ = true;
};

}  // namespace quadropted
