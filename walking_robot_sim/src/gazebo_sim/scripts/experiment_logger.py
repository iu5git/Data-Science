#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
import math
import os
import time
from datetime import datetime


class ExperimentLogger(Node):
    def __init__(self):
        super().__init__("experiment_logger")

        self.declare_parameter("odom_topic", "/robot1/odom")
        self.declare_parameter("output_dir", "/tmp/experiments")

        odom_topic = self.get_parameter("odom_topic").value
        output_dir = self.get_parameter("output_dir").value

        self._last_position = None
        self._start_position = None
        self._start_time = None
        self._total_distance = 0.0
        self._experiment_active = False
        self._trajectory = []
        self._waypoints_count = 0

        os.makedirs(output_dir, exist_ok=True)
        self._output_dir = output_dir

        self._sub_odom = self.create_subscription(
            Odometry, odom_topic, self._odom_callback, 10
        )

        self._srv_start = self.create_service(
            Trigger, "/start_experiment", self._start_callback
        )
        self._srv_stop = self.create_service(
            Trigger, "/stop_experiment", self._stop_callback
        )

        self._log_timer = self.create_timer(1.0, self._log_timer_callback)

        self.get_logger().info(
            f"Experiment Logger ready — odom: {odom_topic}, output: {output_dir}"
        )

    def _odom_callback(self, msg):
        pos = msg.pose.pose.position
        current = (pos.x, pos.y, pos.z)

        if self._experiment_active and self._last_position is not None:
            dx = current[0] - self._last_position[0]
            dy = current[1] - self._last_position[1]
            dz = current[2] - self._last_position[2]
            self._total_distance += math.sqrt(dx * dx + dy * dy + dz * dz)

        self._last_position = current

    def _start_callback(self, request, response):
        if self._experiment_active:
            response.success = False
            response.message = "Experiment already active"
            return response

        if self._last_position is None:
            response.success = False
            response.message = "No odometry data yet"
            return response

        self._experiment_active = True
        self._start_position = self._last_position
        self._start_time = time.time()
        self._total_distance = 0.0
        self._trajectory = [self._last_position]

        self.get_logger().info(
            f"Experiment started at ({self._start_position[0]:.2f}, "
            f"{self._start_position[1]:.2f}, {self._start_position[2]:.2f})"
        )
        response.success = True
        response.message = "Experiment started"
        return response

    def _stop_callback(self, request, response):
        if not self._experiment_active:
            response.success = False
            response.message = "No active experiment"
            return response

        self._experiment_active = False
        end_time = time.time()
        duration = end_time - self._start_time
        end_position = self._last_position

        self._save_results(duration, end_position)

        self.get_logger().info(
            f"Experiment stopped — duration: {duration:.1f}s, "
            f"distance: {self._total_distance:.2f}m"
        )
        response.success = True
        response.message = (
            f"Experiment completed — {duration:.1f}s, "
            f"{self._total_distance:.2f}m"
        )
        return response

    def _save_results(self, duration, end_position):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(self._output_dir, f"experiment_{timestamp}.txt")
        avg_speed = self._total_distance / duration if duration > 0 else 0.0

        with open(filename, "w") as f:
            f.write("=" * 40 + "\n")
            f.write("         EXPERIMENT RESULTS\n")
            f.write("=" * 40 + "\n")
            f.write(f"Date:              {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Duration:          {duration:.1f} sec\n")
            f.write(f"Distance traveled: {self._total_distance:.2f} m\n")
            f.write(f"Average speed:     {avg_speed:.3f} m/s\n")
            f.write(f"Waypoints:         {self._waypoints_count}\n")
            f.write(
                f"Start position:    ({self._start_position[0]:.2f}, "
                f"{self._start_position[1]:.2f}, {self._start_position[2]:.2f})\n"
            )
            if end_position:
                f.write(
                    f"End position:      ({end_position[0]:.2f}, "
                    f"{end_position[1]:.2f}, {end_position[2]:.2f})\n"
                )
            f.write(f"Status:            COMPLETED\n")
            f.write("=" * 40 + "\n")
            f.write("\nTrajectory log (every ~1.0 sec):\n")
            for i, pt in enumerate(self._trajectory):
                t = i * 1.0
                if t > duration:
                    break
                f.write(f"  t={t:.1f}s  pos=({pt[0]:.3f}, {pt[1]:.3f}, {pt[2]:.3f})\n")

        self.get_logger().info(f"Results saved to {filename}")

    def _log_timer_callback(self):
        if self._experiment_active and self._last_position is not None:
            self._trajectory.append(self._last_position)

    def set_waypoints_count(self, count):
        self._waypoints_count = count


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
