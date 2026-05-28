#pragma once
#include <Eigen/Dense>

namespace quadropted {

Eigen::Matrix3d rotx(double alpha);
Eigen::Matrix3d roty(double beta);
Eigen::Matrix3d rotz(double gamma);
Eigen::Matrix3d rotxyz(double alpha, double beta, double gamma);
Eigen::Matrix4d homog_transxyz(double dx, double dy, double dz);
Eigen::Matrix4d homog_transform(double dx, double dy, double dz, double alpha, double beta, double gamma);
Eigen::Matrix4d homog_transform_inverse(const Eigen::Matrix4d& matrix);

}  // namespace quadropted
