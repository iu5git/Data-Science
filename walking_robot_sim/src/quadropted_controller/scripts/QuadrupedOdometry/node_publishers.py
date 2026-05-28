#!/usr/bin/env python3
"""
Публикация Odometry, TF и маркеров.
"""

import tf_transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray


class OdometryPublisher:
    """Миксин для публикации Odometry и TF."""

    def publish_odometry(self):
        """Опубликовать сообщение Odometry и TF."""
        odom = Odometry()
        stamp = self._get_stamp()

        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id

        odom.pose.pose.position.x = self.odom_state.x
        odom.pose.pose.position.y = self.odom_state.y
        odom.pose.pose.position.z = 0.0

        quaternion = tf_transformations.quaternion_from_euler(0, 0, self.odom_state.theta)
        odom.pose.pose.orientation = Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3]
        )

        odom.twist.twist.linear.x = self.odom_state.linear_velocity_x
        odom.twist.twist.linear.y = self.odom_state.linear_velocity_y
        odom.twist.twist.angular.z = self.odom_state.imu_angular_velocity

        self.odom_pub.publish(odom)

        if self.enable_odom_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame_id
            t.child_frame_id = self.base_frame_id

            t.transform.translation.x = self.odom_state.x
            t.transform.translation.y = self.odom_state.y
            t.transform.translation.z = 0.0
            t.transform.rotation = Quaternion(
                x=quaternion[0],
                y=quaternion[1],
                z=quaternion[2],
                w=quaternion[3]
            )

            self.tf_broadcaster.sendTransform(t)

    def _get_stamp(self):
        """Получить timestamp для сообщений."""
        if self.is_gazebo:
            from builtin_interfaces.msg import Time
            return Time(
                sec=self.odom_state.gazebo_clock_sec,
                nanosec=self.odom_state.gazebo_clock_nanosec
            )
        return self.get_clock().now().to_msg()


class MarkerPublisher:
    """Миксин для публикации маркеров позиций лап."""

    def publish_markers(self):
        """Опубликовать маркеры для визуализации лап."""
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        for i, pos in enumerate(self.odom_state.foot_positions):
            marker = Marker()
            marker.header.frame_id = self.base_frame_id
            marker.header.stamp = now
            marker.ns = "foot_markers"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = pos[0]
            marker.pose.position.y = pos[1]
            marker.pose.position.z = pos[2]
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)
