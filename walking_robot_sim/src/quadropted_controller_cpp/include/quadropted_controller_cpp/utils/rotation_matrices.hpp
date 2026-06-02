#pragma once
#include <Eigen/Dense>

namespace quadropted {

Eigen::Matrix3d rotx(double alpha);
Eigen::Matrix3d roty(double beta);
Eigen::Matrix3d rotz(double gamma);
Eigen::Matrix3d rotxyz(double alpha, double beta, double gamma);

}  // namespace quadropted
