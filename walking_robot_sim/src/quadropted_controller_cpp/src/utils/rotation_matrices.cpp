#include "quadropted_controller_cpp/utils/rotation_matrices.hpp"

#include <cmath>

namespace quadropted {

Eigen::Matrix3d rotx(double alpha) {
    double c = std::cos(alpha), s = std::sin(alpha);
    Eigen::Matrix3d m;
    m << 1, 0, 0, 0, c, -s, 0, s, c;
    return m;
}

Eigen::Matrix3d roty(double beta) {
    double c = std::cos(beta), s = std::sin(beta);
    Eigen::Matrix3d m;
    m << c, 0, s, 0, 1, 0, -s, 0, c;
    return m;
}

Eigen::Matrix3d rotz(double gamma) {
    double c = std::cos(gamma), s = std::sin(gamma);
    Eigen::Matrix3d m;
    m << c, -s, 0, s, c, 0, 0, 0, 1;
    return m;
}

Eigen::Matrix3d rotxyz(double alpha, double beta, double gamma) {
    double ca = std::cos(alpha), sa = std::sin(alpha);
    double cb = std::cos(beta), sb = std::sin(beta);
    double cg = std::cos(gamma), sg = std::sin(gamma);
    Eigen::Matrix3d m;
    m << cb * cg, -cb * sg, sb, sa * sb * cg + ca * sg, -sa * sb * sg + ca * cg, -sa * cb, -ca * sb * cg + sa * sg,
        ca * sb * sg + sa * cg, ca * cb;
    return m;
}

}  // namespace quadropted
