#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from quadropted_msgs.msg import DetectionArray


class DetectionVisualizer(Node):
    def __init__(self):
        super().__init__("detection_visualizer")

        self.declare_parameter("detection_topic", "/detections")

        detection_topic = self.get_parameter("detection_topic").value
        self._pub_markers = self.create_publisher(MarkerArray, "detection_markers", 10)
        self._sub = self.create_subscription(
            DetectionArray, detection_topic, self._detections_callback, 10
        )

    def _detections_callback(self, msg):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        for i, d in enumerate(msg.detections):
            marker = Marker()
            marker.header = msg.header
            marker.header.stamp = now
            marker.ns = "detections"
            marker.id = i
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.text = f"{d.class_name} {d.confidence:.2f}"
            marker.pose.position.x = d.center_x
            marker.pose.position.y = d.center_y
            marker.pose.position.z = 0.0
            marker.scale.z = 0.5
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)

            bbox = Marker()
            bbox.header = msg.header
            bbox.header.stamp = now
            bbox.ns = "detections_bbox"
            bbox.id = i
            bbox.type = Marker.LINE_STRIP
            bbox.action = Marker.ADD
            bbox.scale.x = 0.02
            bbox.color.a = 1.0
            bbox.color.r = 0.0
            bbox.color.g = 1.0
            bbox.color.b = 0.0

            hw = d.width / 2.0
            hh = d.height / 2.0
            cx = d.center_x
            cy = d.center_y

            for px, py in [
                (cx - hw, cy - hh),
                (cx + hw, cy - hh),
                (cx + hw, cy + hh),
                (cx - hw, cy + hh),
                (cx - hw, cy - hh),
            ]:
                p = Point()
                p.x = float(px)
                p.y = float(py)
                p.z = 0.0
                bbox.points.append(p)

            marker_array.markers.append(bbox)

        self._pub_markers.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
