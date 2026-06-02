#include "quadropted_controller_cpp/utils/message_builders.hpp"

#include <cmath>

namespace quadropted {

Quaternion build_quaternion_from_yaw(double theta) {
    double half_theta = theta / 2.0;
    return {0.0, 0.0, std::sin(half_theta), std::cos(half_theta)};
}

OdometryData build_odometry_data(double x, double y, double theta, double linear_vx, double linear_vy,
                                 double angular_vz, const std::string& frame_id, const std::string& child_frame_id,
                                 const std::string& stamp) {
    OdometryData data;
    data.header_frame_id = frame_id;
    data.header_stamp = stamp;
    data.child_frame_id = child_frame_id;

    data.pose_position.x = x;
    data.pose_position.y = y;
    data.pose_position.z = 0.0;

    Quaternion q = build_quaternion_from_yaw(theta);
    data.pose_orientation = q;

    data.twist_linear.x = linear_vx;
    data.twist_linear.y = linear_vy;
    data.twist_linear.z = 0.0;

    data.twist_angular.x = 0.0;
    data.twist_angular.y = 0.0;
    data.twist_angular.z = angular_vz;

    return data;
}

TFData build_tf_data(double x, double y, double theta, const std::string& frame_id, const std::string& child_frame_id,
                     const std::string& stamp) {
    TFData data;
    data.header_frame_id = frame_id;
    data.child_frame_id = child_frame_id;
    data.stamp = stamp;

    data.translation.x = x;
    data.translation.y = y;
    data.translation.z = 0.0;

    Quaternion q = build_quaternion_from_yaw(theta);
    data.rotation = q;

    return data;
}

std::vector<MarkerData> build_marker_data(const std::vector<std::array<double, 3>>& foot_positions,
                                          const std::string& frame_id, const std::string& stamp, double marker_scale) {
    std::vector<MarkerData> markers;
    markers.reserve(foot_positions.size());

    // Цвета для каждой ноги: FR-красный, FL-зеленый, RR-синий, RL-желтый
    const double colors[][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 1.0, 0.0}};

    for (size_t i = 0; i < foot_positions.size(); ++i) {
        MarkerData marker;
        marker.frame_id = frame_id;
        marker.stamp = stamp;
        marker.id = static_cast<int>(i);
        marker.position_x = foot_positions[i][0];
        marker.position_y = foot_positions[i][1];
        marker.position_z = foot_positions[i][2];
        marker.scale = marker_scale;
        marker.color_r = colors[i][0];
        marker.color_g = colors[i][1];
        marker.color_b = colors[i][2];
        markers.push_back(marker);
    }

    return markers;
}

}  // namespace quadropted
