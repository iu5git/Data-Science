#!/usr/bin/env python3
import os
import time
from datetime import datetime
import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from quadropted_msgs.msg import Detection, DetectionArray


class YOLODetector(Node):
    def __init__(self):
        super().__init__("yolo_detector")

        self._bridge = CvBridge()
        self._model = None
        self._model_path = None
        self._last_detections = []

        self.declare_parameter("model", "yolov8n.pt")
        self.declare_parameter("fps", 0)
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("camera_topic", "/robot1/color/image_raw")
        self.declare_parameter("target_classes", [])
        self.declare_parameter("device", "cpu")
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("log_interval_sec", 0)
        self.declare_parameter("log_file", "")

        model_name = self.get_parameter("model").value or "yolov8n.pt"

        if "." not in model_name:
            model_name += ".pt"

        models_dir = os.path.join(
            get_package_share_directory("quadropted_perception"), "models"
        )
        local_path = os.path.join(models_dir, model_name)
        if os.path.isfile(local_path):
            resolved_path = local_path
        else:
            resolved_path = model_name
            self.get_logger().warn(
                f"Model not found locally: {local_path}, using ultralytics default"
            )
        self._model_path = resolved_path

        fps = self.get_parameter("fps").value
        self._min_interval = 1.0 / fps if fps > 0 else 0.0
        self._last_time = 0.0

        self._conf = self.get_parameter("confidence_threshold").value
        self._iou = self.get_parameter("iou_threshold").value
        camera_topic = self.get_parameter("camera_topic").value
        self._target_classes = self.get_parameter("target_classes").value
        device = self.get_parameter("device").value
        self._frame_id = self.get_parameter("frame_id").value

        log_interval = float(self.get_parameter("log_interval_sec").value)
        log_file = self.get_parameter("log_file").value
        self._log_file = log_file if log_file else None
        self._log_buffer = ""

        if self._log_file:
            os.makedirs(os.path.dirname(self._log_file) or ".", exist_ok=True)
            with open(self._log_file, "w") as f:
                f.write("timestamp,class_id,class_name,confidence,center_x,center_y,width,height\n")
            self.get_logger().info(f"Detection logging enabled → {self._log_file}")

        if log_interval > 0 and self._log_file:
            self.create_timer(log_interval, self._log_timer_callback)

        self.get_logger().info(f"Loading YOLO model: {resolved_path} (device: {device})")
        self._model = YOLO(resolved_path)
        self._model.to(device)

        self._pub_detections = self.create_publisher(DetectionArray, "detections", 10)
        self._pub_debug_image = self.create_publisher(Image, "detected_image", 10)

        self._sub_camera = self.create_subscription(
            Image, camera_topic, self._image_callback, 10
        )

        self.get_logger().info(
            f"YOLO detector ready — model: {self._model_path}, topic: {camera_topic}, "
            f"conf: {self._conf}, iou: {self._iou}"
        )
        if self._min_interval > 0:
            self.get_logger().info(f"Throttling to {fps} FPS (min interval: {self._min_interval:.3f}s)")

    def _image_callback(self, msg):
        now = time.monotonic()
        if self._min_interval > 0 and (now - self._last_time) < self._min_interval:
            return
        self._last_time = now

        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        results = self._model(
            cv_image,
            conf=self._conf,
            iou=self._iou,
            classes=self._target_classes if self._target_classes else None,
            verbose=False,
        )[0]

        detections_msg = DetectionArray()
        detections_msg.header = msg.header
        detections_msg.header.frame_id = self._frame_id

        if results.boxes is not None:
            for box in results.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                x1, y1, x2, y2 = map(float, box.xyxy[0])

                d = Detection()
                d.class_id = cls_id
                d.class_name = results.names[cls_id]
                d.confidence = conf
                d.center_x = (x1 + x2) / 2.0
                d.center_y = (y1 + y2) / 2.0
                d.width = x2 - x1
                d.height = y2 - y1
                detections_msg.detections.append(d)

        self._pub_detections.publish(detections_msg)
        self._last_detections = detections_msg.detections

        annotated = results.plot()
        try:
            debug_msg = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            debug_msg.header = msg.header
            self._pub_debug_image.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish debug image: {e}")

    def _log_timer_callback(self):
        if not self._log_file:
            return
        timestamp = datetime.now().strftime("%Y-%m-%d_%H:%M:%S.%f")[:23]
        with open(self._log_file, "a") as f:
            if not self._last_detections:
                f.write(f"{timestamp},,,,no detections\n")
                return
            for d in self._last_detections:
                f.write(
                    f"{timestamp},{d.class_id},{d.class_name},{d.confidence:.3f},"
                    f"{d.center_x:.1f},{d.center_y:.1f},{d.width:.1f},{d.height:.1f}\n"
                )

    @property
    def model_path(self):
        return self._model_path


def main(args=None):
    rclpy.init(args=args)
    node = YOLODetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
