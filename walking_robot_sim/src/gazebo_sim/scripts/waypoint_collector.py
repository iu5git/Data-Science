#!/usr/bin/env python3
# pyright: reportAttributeAccessIssue=false, reportOptionalMemberAccess=false

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import FollowWaypoints
from nav2_simple_commander.robot_navigator import BasicNavigator
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger
from quadropted_msgs.srv import WaypointNavigate, LoadWaypoints, GetWaypoints
from quadropted_msgs.msg import Waypoint
from action_msgs.msg import GoalStatus
import threading
import json
import math
import os
import yaml


def _get_waypoints_dir():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(
            script_dir, "..", "..", "share", "gazebo_sim", "config", "waypoints"
        ),
        os.path.join(script_dir, "..", "config", "waypoints"),
    ]
    candidate = None
    dir_name = os.path.dirname(script_dir)
    while True:
        candidate = os.path.join(dir_name, "config", "waypoints")
        if os.path.isdir(candidate):
            return candidate
        parent = os.path.dirname(dir_name)
        if parent == dir_name:
            break
        dir_name = parent
    for c in candidates:
        c = os.path.normpath(c)
        if os.path.isdir(c):
            return c
    return os.path.normpath(candidates[0])


class WaypointCollector(Node):
    def __init__(self):
        super().__init__("waypoint_collector")
        self.waypoints = []
        ns = self.get_namespace().lstrip("/")
        self.navigator = BasicNavigator(namespace=ns)

        # Own ActionClient on this node (which is in the executor)
        self._follow_wp_client = ActionClient(self, FollowWaypoints, "follow_waypoints")

        self.navigation_active = False
        self._nav_goal_handle = None
        self._nav_result_future = None
        self._pending_goal = None
        self._goal_retry_timer = None
        self._current_waypoint_index = 0
        self._resume_index = 0
        self._resume_offset = 0

        self.subscription = self.create_subscription(
            PoseStamped, "/custom_goal_pose", self.goal_pose_callback, 10
        )

        self.waypoint_publisher = self.create_publisher(
            PoseArray, "/custom_waypoints", 10
        )

        self.marker_publisher = self.create_publisher(
            MarkerArray, "/waypoint_markers", 10
        )

        self.clear_service = self.create_service(
            Trigger, "/clear_waypoints", self.clear_waypoints_callback
        )

        self.start_service = self.create_service(
            Trigger, "/start_navigation", self.start_navigation_callback
        )

        self.navigate_service = self.create_service(
            WaypointNavigate,
            "/navigate_to_waypoint",
            self.navigate_to_waypoint_callback,
        )

        self.stop_service = self.create_service(
            Trigger, "/stop_navigation", self.stop_navigation_callback
        )

        self.resume_service = self.create_service(
            Trigger, "/resume_navigation", self.resume_navigation_callback
        )

        self.load_service = self.create_service(
            LoadWaypoints, "/load_waypoints", self.load_waypoints_callback
        )

        self.get_service = self.create_service(
            GetWaypoints, "/get_waypoints", self.get_waypoints_callback
        )

        self.timer = self.create_timer(0.1, self.check_navigation)

        self.nav2_ready = False
        self._start_nav2_wait_thread()

        self.get_logger().info("Waypoint Collector Node started")

    def goal_pose_callback(self, msg):
        self.waypoints.append(msg)
        self.get_logger().info(
            f"Added waypoint: x={msg.pose.position.x}, y={msg.pose.position.y}, total waypoints: {len(self.waypoints)}"
        )
        self.publish_markers()

    def publish_markers(self):
        marker_array = MarkerArray()

        for i, wp in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = wp.pose
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            color = ColorRGBA()
            if i % 3 == 0:
                color.r, color.g, color.b, color.a = 1.0, 0.0, 0.0, 1.0
            elif i % 3 == 1:
                color.r, color.g, color.b, color.a = 0.0, 1.0, 0.0, 1.0
            else:
                color.r, color.g, color.b, color.a = 0.0, 0.0, 1.0, 1.0

            marker.color = color
            marker_array.markers.append(marker)

            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "waypoint_labels"
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = wp.pose.position.x + 0.3
            text_marker.pose.position.y = wp.pose.position.y
            text_marker.pose.position.z = wp.pose.position.z + 0.3
            text_marker.pose.orientation = wp.pose.orientation
            text_marker.scale.z = 0.25
            text_marker.color = color
            text_marker.text = str(i)
            marker_array.markers.append(text_marker)

        self.marker_publisher.publish(marker_array)
        self.get_logger().info(
            f"Published {len(self.waypoints)} markers to /waypoint_markers"
        )

        pose_array = PoseArray()
        pose_array.header.frame_id = "map"
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.poses = [wp.pose for wp in self.waypoints]
        self.waypoint_publisher.publish(pose_array)

    def clear_waypoints_callback(self, request, response):
        try:
            if self.navigation_active:
                self.cancel_navigation()
        except Exception as e:
            self.get_logger().error(f"Failed to cancel navigation task: {str(e)}")

        self.waypoints = []
        self.navigation_active = False
        self._resume_index = 0
        self._resume_offset = 0
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "waypoints"
        marker.id = 0
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)
        label_marker = Marker()
        label_marker.header.frame_id = "map"
        label_marker.header.stamp = self.get_clock().now().to_msg()
        label_marker.ns = "waypoint_labels"
        label_marker.id = 0
        label_marker.action = Marker.DELETEALL
        marker_array.markers.append(label_marker)
        self.marker_publisher.publish(marker_array)
        self.get_logger().info("Waypoints and markers cleared via service call")
        response.success = True
        response.message = "Waypoints cleared successfully"
        return response

    def _yaw_to_quat(self, yaw):
        return {
            "x": 0.0,
            "y": 0.0,
            "z": math.sin(yaw / 2.0),
            "w": math.cos(yaw / 2.0),
        }

    def load_waypoints_callback(self, request, response):
        file_path = request.file_path
        if not file_path:
            file_path = "default.yaml"

        if not os.path.isabs(file_path):
            file_path = os.path.join(_get_waypoints_dir(), file_path)

        data = None
        _, ext = os.path.splitext(file_path)
        if not ext:
            yaml_path = file_path + ".yaml"
            if os.path.exists(yaml_path):
                file_path = yaml_path
                ext = ".yaml"
            else:
                json_path = file_path + ".json"
                if os.path.exists(json_path):
                    file_path = json_path
                    ext = ".json"
        try:
            if ext in (".yaml", ".yml"):
                with open(file_path, "r") as f:
                    data = yaml.safe_load(f)
            else:
                with open(file_path, "r") as f:
                    data = json.load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to read file {file_path}: {e}")
            response.success = False
            response.message = f"Failed to read file {file_path}: {e}"
            return response

        if not isinstance(data, list):
            response.success = False
            response.message = "Waypoints file must be an array"
            return response

        if self.navigation_active:
            self.cancel_navigation()

        self.waypoints = []
        for i, wp_data in enumerate(data):
            pose = Pose()
            pose.position.x = float(wp_data.get("x", 0.0))
            pose.position.y = float(wp_data.get("y", 0.0))
            pose.position.z = float(wp_data.get("z", 0.0))

            if "yaw" in wp_data:
                quat = self._yaw_to_quat(float(wp_data["yaw"]))
                pose.orientation.x = quat["x"]
                pose.orientation.y = quat["y"]
                pose.orientation.z = quat["z"]
                pose.orientation.w = quat["w"]
            else:
                pose.orientation.w = 1.0

            stamped = PoseStamped()
            stamped.header.frame_id = "map"
            stamped.header.stamp = self.get_clock().now().to_msg()
            stamped.pose = pose
            self.waypoints.append(stamped)

        self.get_logger().info(
            f"Loaded {len(self.waypoints)} waypoints from {file_path}"
        )
        self.publish_markers()
        response.success = True
        response.message = f"Loaded {len(self.waypoints)} waypoints from {file_path}"
        return response

    def get_waypoints_callback(self, request, response):
        response.success = True
        response.message = f"{len(self.waypoints)} waypoints"
        response.waypoints = []
        for wp in self.waypoints:
            w = Waypoint()
            w.x = wp.pose.position.x
            w.y = wp.pose.position.y
            w.z = wp.pose.position.z
            q = wp.pose.orientation
            w.yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            response.waypoints.append(w)
        return response

    def start_navigation_callback(self, request, response):
        if not self.waypoints:
            self.get_logger().warn("No waypoints available to start navigation")
            response.success = False
            response.message = "No waypoints to navigate"
            return response

        if self.navigation_active:
            self.get_logger().warn("Navigation is already active")
            response.success = False
            response.message = "Navigation is already active"
            return response

        if not self.nav2_ready:
            self.get_logger().warn("Nav2 is not ready yet, waiting...")
            response.success = False
            response.message = "Nav2 is not ready yet"
            return response

        try:
            pose_array = PoseArray()
            pose_array.header.frame_id = "map"
            pose_array.header.stamp = self.get_clock().now().to_msg()
            pose_array.poses = [wp.pose for wp in self.waypoints]
            self.waypoint_publisher.publish(pose_array)
            self.get_logger().info(
                f"Published {len(self.waypoints)} waypoints to /custom_waypoints"
            )

            self.navigation_active = True
            self._resume_index = 0
            self._resume_offset = 0
            self._send_goal_async(self.waypoints)
            self.get_logger().info(
                f"Sent {len(self.waypoints)} waypoints to FollowWaypoints action"
            )
            response.success = True
            response.message = "Navigation started successfully"
        except Exception as e:
            self.get_logger().error(f"Failed to start navigation: {str(e)}")
            self.navigation_active = False
            response.success = False
            response.message = f"Failed to start navigation: {str(e)}"
        return response

    def _make_goal_msg(self, waypoints):
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints
        return goal_msg

    def _send_goal_async(self, waypoints=None):
        if waypoints is None:
            waypoints = self.waypoints
        goal_msg = self._make_goal_msg(waypoints)

        if self._follow_wp_client.server_is_ready():
            self._do_send_goal(goal_msg)
        else:
            self.get_logger().warn("FollowWaypoints server not ready, retrying...")
            self._pending_goal = goal_msg
            self._goal_retry_timer = self.create_timer(0.5, self._retry_send_goal)

    def _retry_send_goal(self):
        if self._follow_wp_client.server_is_ready():
            self._goal_retry_timer.cancel()
            self._do_send_goal(self._pending_goal)
            self._pending_goal = None
        else:
            self.get_logger().info("Still waiting for FollowWaypoints server...")

    def _do_send_goal(self, goal_msg):
        send_goal_future = self._follow_wp_client.send_goal_async(
            goal_msg, self._feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _feedback_callback(self, feedback_msg):
        self._current_waypoint_index = (
            self._resume_offset + feedback_msg.feedback.current_waypoint
        )
        self.get_logger().info(f"Current waypoint: {self._current_waypoint_index}")

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("FollowWaypoints goal was rejected")
            self.navigation_active = False
            return

        self._nav_goal_handle = goal_handle
        self.get_logger().info("FollowWaypoints goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result()
        self._nav_result_future = future
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Navigation SUCCEEDED")
            self._resume_index = len(self.waypoints)
        elif result.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Navigation FAILED (aborted)")
        elif result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Navigation CANCELED")
        else:
            self.get_logger().warn(f"Navigation result status: {result.status}")
        self.navigation_active = False

    def cancel_navigation(self):
        if self._nav_goal_handle:
            self._nav_goal_handle.cancel_goal_async()
        self._resume_index = self._current_waypoint_index
        self.navigation_active = False

    def navigate_to_waypoint_callback(self, request, response):
        idx = request.index

        if not self.waypoints:
            self.get_logger().warn("No waypoints available")
            response.success = False
            response.message = "No waypoints to navigate"
            return response

        if idx < -1 or idx >= len(self.waypoints):
            self.get_logger().error(
                f"Invalid waypoint index {idx}, have {len(self.waypoints)} waypoints"
            )
            response.success = False
            response.message = (
                f"Invalid index {idx}, valid: -1..{len(self.waypoints) - 1}"
            )
            return response

        if self.navigation_active:
            self.cancel_navigation()
            self.get_logger().info("Cancelled previous navigation")

        if not self.nav2_ready:
            self.get_logger().warn("Nav2 is not ready yet")
            response.success = False
            response.message = "Nav2 is not ready yet"
            return response

        try:
            if idx == -1:
                targets = self.waypoints
                label = "all"
            else:
                targets = [self.waypoints[idx]]
                label = str(idx)

            self.navigation_active = True
            self._resume_offset = idx if idx != -1 else 0
            self._send_goal_async(targets)
            self.get_logger().info(f"Navigating to waypoint index {label}")
            response.success = True
            response.message = f"Navigating to waypoint index {label}"
        except Exception as e:
            self.get_logger().error(f"Failed to start navigation: {str(e)}")
            self.navigation_active = False
            response.success = False
            response.message = str(e)
        return response

    def stop_navigation_callback(self, request, response):
        try:
            self.cancel_navigation()
            self.get_logger().info("Navigation stopped via /stop_navigation")
            response.success = True
            response.message = "Navigation stopped"
        except Exception as e:
            self.get_logger().error(f"Failed to stop navigation: {str(e)}")
            response.success = False
            response.message = str(e)
        return response

    def resume_navigation_callback(self, request, response):
        if not self.waypoints:
            self.get_logger().warn("No waypoints to resume")
            response.success = False
            response.message = "No waypoints to navigate"
            return response

        if self.navigation_active:
            self.get_logger().warn("Navigation is already active")
            response.success = False
            response.message = "Navigation is already active"
            return response

        if not self.nav2_ready:
            self.get_logger().warn("Nav2 is not ready yet")
            response.success = False
            response.message = "Nav2 is not ready yet"
            return response

        remaining = self.waypoints[self._resume_index :]
        if not remaining:
            self.get_logger().warn("No remaining waypoints to resume")
            response.success = False
            response.message = "No remaining waypoints"
            return response

        try:
            self.navigation_active = True
            self._resume_offset = self._resume_index
            self._send_goal_async(remaining)
            self.get_logger().info(
                f"Resumed navigation from waypoint {self._resume_index} "
                f"({len(remaining)} remaining)"
            )
            response.success = True
            response.message = (
                f"Resumed from waypoint {self._resume_index}, "
                f"{len(remaining)} remaining"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to resume navigation: {str(e)}")
            self.navigation_active = False
            response.success = False
            response.message = str(e)
        return response

    def check_navigation(self):
        if self.navigation_active and self._nav_result_future:
            if self._nav_result_future.result():
                status = self._nav_result_future.result().status
                if (
                    status != GoalStatus.STATUS_SUCCEEDED
                    and status != GoalStatus.STATUS_EXECUTING
                ):
                    self.get_logger().info(
                        f"Navigation completed with status: {status}"
                    )
                    self.navigation_active = False

    def _start_nav2_wait_thread(self):
        thread = threading.Thread(target=self._wait_for_nav2, daemon=True)
        thread.start()

    def _wait_for_nav2(self):
        try:
            self.get_logger().info("Waiting for Nav2 to become active...")
            self.navigator.waitUntilNav2Active()
            self.nav2_ready = True
            self.get_logger().info("Nav2 is active")
        except Exception as e:
            self.get_logger().error(f"Failed to activate Nav2: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointCollector()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
