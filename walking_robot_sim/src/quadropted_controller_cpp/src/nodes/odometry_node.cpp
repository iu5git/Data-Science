#include <tf2_ros/transform_broadcaster.h>

#include <array>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quadropted_msgs/msg/robot_foot_contact.hpp>
#include <quadropted_msgs/msg/robot_velocity.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "quadropted_controller_cpp/kinematics/forward_kinematics.hpp"
#include "quadropted_controller_cpp/odometry/odometry.hpp"
#include "quadropted_controller_cpp/utils/message_builders.hpp"

class DogOdometryNode : public rclcpp::Node {
  public:
    DogOdometryNode() : Node("dog_odometry") {
        // Параметры
        declare_parameter("verbose", false);
        declare_parameter("publish_rate", 50);
        declare_parameter("has_imu_heading", true);
        declare_parameter("enable_odom_tf", false);
        declare_parameter("base_frame_id", "base");
        declare_parameter("odom_frame_id", "odom");
        declare_parameter("is_gazebo", true);
        declare_parameter("filter_window_size", 14);
        declare_parameter("imu_topic", "imu_plugin/out");

        verbose_ = get_parameter("verbose").as_bool();
        publish_rate_ = get_parameter("publish_rate").as_int();
        has_imu_heading_ = get_parameter("has_imu_heading").as_bool();
        enable_odom_tf_ = get_parameter("enable_odom_tf").as_bool();
        base_frame_id_ = get_parameter("base_frame_id").as_string();
        odom_frame_id_ = get_parameter("odom_frame_id").as_string();
        is_gazebo_ = get_parameter("is_gazebo").as_bool();
        int filter_window = static_cast<int>(get_parameter("filter_window_size").as_int());
        std::string imu_topic = get_parameter("imu_topic").as_string();

        // FK solver
        double body_length = 0.3762, body_width = 0.0935;
        double l1 = 0.0, l2 = 0.0955, l3 = 0.213, l4 = 0.213;
        fk_ = std::make_unique<quadropted::ForwardKinematics>(body_length, body_width, l1, l2, l3, l4);

        // Состояние
        odom_state_ = std::make_unique<quadropted::OdometryState>(filter_window);
        last_position_time_ = now();

        // Publishers
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("foot_markers", 10);

        // TF broadcaster
        if (enable_odom_tf_) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

        // Subscriptions
        if (has_imu_heading_) {
            imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
                imu_topic, 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) { imu_callback(msg); });
        }

        joint_states_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "joint_group_controller/commands", 10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { joint_states_callback(msg); });

        foot_contacts_sub_ = create_subscription<quadropted_msgs::msg::RobotFootContact>(
            "foot_contact", rclcpp::SensorDataQoS(),
            [this](const quadropted_msgs::msg::RobotFootContact::SharedPtr msg) { foot_contacts_callback(msg); });

        velocity_sub_ = create_subscription<quadropted_msgs::msg::RobotVelocity>(
            "robot_velocity", 10, [this](const quadropted_msgs::msg::RobotVelocity::SharedPtr msg) {
                if (msg->robot_id == 1) {
                    odom_state_->linear_velocity_x = msg->cmd_vel.linear.x;
                    odom_state_->linear_velocity_y = msg->cmd_vel.linear.y;
                }
            });

        // Timer
        double timer_period = 1.0 / static_cast<double>(publish_rate_);
        timer_ = create_wall_timer(std::chrono::duration<double>(timer_period), [this]() { timer_callback(); });

        RCLCPP_INFO(get_logger(), "Dog Odometry Node (C++) has been started.");
    }

  private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        double qx = msg->orientation.x;
        double qy = msg->orientation.y;
        double qz = msg->orientation.z;
        double qw = msg->orientation.w;

        // Euler from quaternion (roll, pitch, yaw) — упрощённо только yaw
        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        double yaw = std::atan2(siny_cosp, cosy_cosp);

        odom_state_->theta = quadropted::normalize_angle(yaw);
        odom_state_->imu_angular_velocity = -msg->angular_velocity.z;
    }

    void joint_states_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() != 12) {
            RCLCPP_ERROR(get_logger(), "Unexpected number of joint angles: %zu. Expected 12.", msg->data.size());
            return;
        }
        for (size_t i = 0; i < 12; ++i) {
            odom_state_->joint_positions[i] = msg->data[i];
        }
    }

    void foot_contacts_callback(const quadropted_msgs::msg::RobotFootContact::SharedPtr msg) {
        if (msg->contacts.size() != 4) {
            RCLCPP_ERROR(get_logger(), "Unexpected number of contacts: %zu. Expected 4.", msg->contacts.size());
            for (int i = 0; i < 4; ++i)
                odom_state_->foot_contacts[i] = false;
            return;
        }
        for (int i = 0; i < 4; ++i) {
            odom_state_->foot_contacts[i] = msg->contacts[i];
        }
    }

    void calculate_foot_positions() {
        try {
            std::vector<double> joints(12);
            for (int i = 0; i < 12; ++i)
                joints[i] = odom_state_->joint_positions[i];
            auto foot_positions = fk_->forward_kinematics_all_legs(joints);
            for (int i = 0; i < 4; ++i) {
                odom_state_->foot_positions[i] = foot_positions[i];
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Error in forward kinematics: %s", e.what());
            for (int i = 0; i < 4; ++i)
                odom_state_->foot_positions[i] = Eigen::Vector3d::Zero();
        }
    }

    void update_odometry_step() {
        rclcpp::Time current_time = now();
        double dt = (current_time - last_position_time_).seconds();
        if (dt <= 0.0) return;

        quadropted::update_odometry(*odom_state_, dt);
        last_position_time_ = current_time;
    }

    void publish_odometry() {
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now();
        odom_msg.header.frame_id = odom_frame_id_;
        odom_msg.child_frame_id = base_frame_id_;

        odom_msg.pose.pose.position.x = odom_state_->x;
        odom_msg.pose.pose.position.y = odom_state_->y;
        odom_msg.pose.pose.position.z = 0.0;

        double half_theta = odom_state_->theta / 2.0;
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = std::sin(half_theta);
        odom_msg.pose.pose.orientation.w = std::cos(half_theta);

        odom_msg.twist.twist.linear.x = odom_state_->linear_velocity_x;
        odom_msg.twist.twist.linear.y = odom_state_->linear_velocity_y;
        odom_msg.twist.twist.angular.z = odom_state_->imu_angular_velocity;

        odom_pub_->publish(odom_msg);

        if (enable_odom_tf_) {
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = odom_msg.header.stamp;
            tf_msg.header.frame_id = odom_frame_id_;
            tf_msg.child_frame_id = base_frame_id_;
            tf_msg.transform.translation.x = odom_state_->x;
            tf_msg.transform.translation.y = odom_state_->y;
            tf_msg.transform.translation.z = 0.0;
            tf_msg.transform.rotation = odom_msg.pose.pose.orientation;
            tf_broadcaster_->sendTransform(tf_msg);
        }
    }

    void publish_markers() {
        visualization_msgs::msg::MarkerArray marker_array;
        auto now_stamp = now();

        const double colors[4][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 1.0, 0.0}};

        for (int i = 0; i < 4; ++i) {
            visualization_msgs::msg::Marker marker;
            marker.header.stamp = now_stamp;
            marker.header.frame_id = base_frame_id_;
            marker.ns = "foot_markers";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = odom_state_->foot_positions[i].x();
            marker.pose.position.y = odom_state_->foot_positions[i].y();
            marker.pose.position.z = odom_state_->foot_positions[i].z();
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            marker.color.a = 1.0;
            marker.color.r = colors[i][0];
            marker.color.g = colors[i][1];
            marker.color.b = colors[i][2];
            marker_array.markers.push_back(marker);
        }

        marker_pub_->publish(marker_array);
    }

    void timer_callback() {
        calculate_foot_positions();
        update_odometry_step();
        publish_odometry();
        publish_markers();
    }

    // Members
    std::unique_ptr<quadropted::ForwardKinematics> fk_;
    std::unique_ptr<quadropted::OdometryState> odom_state_;
    rclcpp::Time last_position_time_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<quadropted_msgs::msg::RobotFootContact>::SharedPtr foot_contacts_sub_;
    rclcpp::Subscription<quadropted_msgs::msg::RobotVelocity>::SharedPtr velocity_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    bool verbose_ = false;
    int publish_rate_ = 50;
    bool has_imu_heading_ = true;
    bool enable_odom_tf_ = false;
    std::string base_frame_id_ = "base";
    std::string odom_frame_id_ = "odom";
    bool is_gazebo_ = true;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DogOdometryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
