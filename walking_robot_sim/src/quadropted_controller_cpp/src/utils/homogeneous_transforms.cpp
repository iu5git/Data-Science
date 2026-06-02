#include "quadropted_controller_cpp/utils/homogeneous_transforms.hpp"

namespace quadropted {

Eigen::Matrix4d homog_transxyz(double dx, double dy, double dz) {
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    m(0, 3) = dx;
    m(1, 3) = dy;
    m(2, 3) = dz;
    return m;
}

Eigen::Matrix4d homog_transform(double dx, double dy, double dz, double alpha, double beta, double gamma) {
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    m.block<3, 3>(0, 0) = rotxyz(alpha, beta, gamma);
    m(0, 3) = dx;
    m(1, 3) = dy;
    m(2, 3) = dz;
    return m;
}

Eigen::Matrix4d homog_transform_inverse(const Eigen::Matrix4d& matrix) {
    Eigen::Matrix4d inv = Eigen::Matrix4d::Identity();
    inv.block<3, 3>(0, 0) = matrix.block<3, 3>(0, 0).transpose();
    inv.block<3, 1>(0, 3) = -inv.block<3, 3>(0, 0) * matrix.block<3, 1>(0, 3);
    return inv;
}

}  // namespace quadropted
