#include "quadropted_controller_cpp/kinematics/inverse_kinematics.hpp"

#include <cmath>

namespace quadropted {

Eigen::MatrixXd compute_local_positions(const Eigen::MatrixXd& leg_positions, double body_length, double body_width,
                                        double dx, double dy, double dz, double roll, double pitch, double yaw) {
    // Фиксированная матрица вращения для ног: R = rotxyz(pi/2, -pi/2, 0)
    Eigen::Matrix3d R_legs;
    R_legs << 0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0;

    // T_blwbl — преобразование корпуса
    Eigen::Matrix4d T_blwbl = Eigen::Matrix4d::Identity();
    T_blwbl.block<3, 3>(0, 0) = rotxyz(roll, pitch, yaw);
    T_blwbl(0, 3) = dx;
    T_blwbl(1, 3) = dy;
    T_blwbl(2, 3) = dz;

    double hl = 0.5 * body_length;
    double hw = 0.5 * body_width;

    // Матрицы преобразования для каждой ноги
    auto make_leg_T = [&](double tx, double ty, double tz) {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = R_legs;
        T(0, 3) = tx;
        T(1, 3) = ty;
        T(2, 3) = tz;
        return T;
    };

    Eigen::Matrix4d T_leg[4];
    T_leg[0] = T_blwbl * make_leg_T(hl, -hw, 0);   // FR
    T_leg[1] = T_blwbl * make_leg_T(hl, hw, 0);    // FL
    T_leg[2] = T_blwbl * make_leg_T(-hl, -hw, 0);  // RR
    T_leg[3] = T_blwbl * make_leg_T(-hl, hw, 0);   // RL

    // Обратное преобразование для каждой ноги
    Eigen::MatrixXd result(4, 3);
    for (int i = 0; i < 4; ++i) {
        Eigen::Matrix4d inv_T = homog_transform_inverse(T_leg[i]);
        Eigen::Vector4d leg_pos_h;
        leg_pos_h << leg_positions.col(i), 1.0;
        Eigen::Vector4d pos_local = inv_T * leg_pos_h;
        result.row(i) = pos_local.head<3>();
    }

    return result;
}

std::array<double, 3> compute_joint_angles_for_leg(double x, double y, double z, int leg_index, double l1, double l2,
                                                   double l3, double l4) {
    static const double LEG_SIGNS[] = {1.0, -1.0, 1.0, -1.0};

    double l2_sq = l2 * l2;
    double f_sq = x * x + y * y - l2_sq;
    double F = (f_sq > 0.0) ? std::sqrt(f_sq) : 0.0;
    double G = F - l1;
    double H = std::sqrt(G * G + z * z);

    double theta1 = -std::atan2(y, x) - std::atan2(F, l2 * LEG_SIGNS[leg_index]);

    double _2l3l4 = 2.0 * l3 * l4;
    double l3sq_l4sq = l3 * l3 + l4 * l4;
    double D = (H * H - l3sq_l4sq) / _2l3l4;
    if (D > 1.0)
        D = 1.0;
    else if (D < -1.0)
        D = -1.0;

    double theta4 = -std::atan2(std::sqrt(1.0 - D * D), D);
    double theta3 = std::atan2(z, G) - std::atan2(l4 * std::sin(theta4), l3 + l4 * std::cos(theta4));

    return {theta1, theta3, theta4};
}

std::vector<double> compute_all_joint_angles(const Eigen::MatrixXd& positions, double l1, double l2, double l3,
                                             double l4) {
    static const double LEG_SIGNS[] = {1.0, -1.0, 1.0, -1.0};

    double l2_sq = l2 * l2;
    double _2l3l4 = 2.0 * l3 * l4;
    double inv_2l3l4 = 1.0 / _2l3l4;
    double l3sq_l4sq = l3 * l3 + l4 * l4;

    std::vector<double> angles(12, 0.0);

    for (int i = 0; i < 4; ++i) {
        double x = positions(0, i);
        double y = positions(1, i);
        double z = positions(2, i);

        double f_sq = x * x + y * y - l2_sq;
        double F = (f_sq > 0.0) ? std::sqrt(f_sq) : 0.0;
        double G = F - l1;
        double H = std::sqrt(G * G + z * z);

        double theta1 = -std::atan2(y, x) - std::atan2(F, l2 * LEG_SIGNS[i]);

        double D = (H * H - l3sq_l4sq) * inv_2l3l4;
        if (D > 1.0)
            D = 1.0;
        else if (D < -1.0)
            D = -1.0;

        double theta4 = -std::atan2(std::sqrt(1.0 - D * D), D);
        double theta3 = std::atan2(z, G) - std::atan2(l4 * std::sin(theta4), l3 + l4 * std::cos(theta4));

        int idx = i * 3;
        angles[idx] = theta1;
        angles[idx + 1] = theta3;
        angles[idx + 2] = theta4;
    }

    return angles;
}

InverseKinematics::InverseKinematics(double body_length, double body_width, double l1, double l2, double l3, double l4)
    : fk_(body_length, body_width, l1, l2, l3, l4),
      body_length_(body_length),
      body_width_(body_width),
      l1_(l1),
      l2_(l2),
      l3_(l3),
      l4_(l4) {}

Eigen::MatrixXd InverseKinematics::get_local_positions(const Eigen::MatrixXd& leg_positions, double dx, double dy,
                                                       double dz, double roll, double pitch, double yaw) const {
    return compute_local_positions(leg_positions, body_length_, body_width_, dx, dy, dz, roll, pitch, yaw);
}

std::vector<double> InverseKinematics::inverse_kinematics(const Eigen::MatrixXd& leg_positions, double dx, double dy,
                                                          double dz, double roll, double pitch, double yaw) const {
    Eigen::MatrixXd positions = get_local_positions(leg_positions, dx, dy, dz, roll, pitch, yaw);
    // compute_local_positions возвращает (4, 3), но compute_all_joint_angles
    // ожидает (3, 4) — транспонируем
    return compute_all_joint_angles(positions.transpose(), l1_, l2_, l3_, l4_);
}

}  // namespace quadropted
