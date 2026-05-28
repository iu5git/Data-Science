#pragma once
#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace quadropted {
struct Quaternion {
    double x, y, z, w;
};
struct Position {
    double x, y, z;
};
struct TwistLin {
    double x, y, z;
};
struct TwistAng {
    double x, y, z;
};
struct OdometryData {
    std::string header_frame_id, header_stamp, child_frame_id;
    Position pose_position;
    Quaternion pose_orientation;
    TwistLin twist_linear;
    TwistAng twist_angular;
};
struct TFData {
    std::string header_frame_id, child_frame_id, stamp;
    Position translation;
    Quaternion rotation;
};
struct MarkerData {
    std::string frame_id, stamp;
    int id;
    double position_x, position_y, position_z, scale;
    double color_r, color_g, color_b;
};

Quaternion build_quaternion_from_yaw(double theta);
OdometryData build_odometry_data(double x, double y, double theta, double linear_vx, double linear_vy,
                                 double angular_vz, const std::string& frame_id, const std::string& child_frame_id,
                                 const std::string& stamp);
TFData build_tf_data(double x, double y, double theta, const std::string& frame_id, const std::string& child_frame_id,
                     const std::string& stamp);
std::vector<MarkerData> build_marker_data(const std::vector<std::array<double, 3>>& foot_positions,
                                          const std::string& frame_id, const std::string& stamp,
                                          double marker_scale = 0.05);
}  // namespace quadropted
