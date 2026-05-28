#!/usr/bin/env python3
"""
Multi Action Server

Этот сервер реализует два action‑сервера:
  1. На топике "test_action": вычисляет последовательность Фибоначчи.
  2. На топике "count_action": ожидает указанное время и возвращает результат (используя Wait).
"""

import time

import rclpy
from builtin_interfaces.msg import Duration
from example_interfaces.action import Fibonacci
from nav2_msgs.action import Wait
from rclpy.action import ActionServer
from rclpy.node import Node


class MultiActionServer(Node):
    def __init__(self):
        super().__init__('multi_action_server')

        # Action Server 1: test_action (вычисление Фибоначчи)
        self._action_server_test = ActionServer(
            self,
            Fibonacci,
            'test_action',
            self.execute_test_callback
        )
        self.get_logger().info("Test Action Server (test_action) started.")

        # Action Server 2: count_action (ожидание указанного времени)
        self._action_server_count = ActionServer(
            self,
            Wait,
            'count_action',
            self.execute_count_callback
        )
        self.get_logger().info("Count Action Server (count_action) started.")

    def execute_test_callback(self, goal_handle):
        self.get_logger().info("Received goal request on test_action")
        order = goal_handle.request.order
        self.get_logger().info(f"Goal order: {order}")

        feedback_msg = Fibonacci.Feedback()
        sequence = [0, 1]
        try:
            feedback_msg.sequence = sequence
        except AttributeError as e:
            self.get_logger().error(f"AttributeError: {e}")
            return Fibonacci.Result()

        # Вычисление последовательности Фибоначчи
        for i in range(1, order):
            next_number = sequence[i] + sequence[i - 1]
            sequence.append(next_number)
            try:
                feedback_msg.sequence = sequence
            except AttributeError:
                self.get_logger().error("Failed to update feedback message attribute 'sequence'")
            self.get_logger().info(f"test_action Feedback: {sequence}")
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = sequence
        self.get_logger().info(f"test_action Returning result: {sequence}")
        return result

    def execute_count_callback(self, goal_handle):
        self.get_logger().info("Received goal request on count_action")
        # Для Wait-цели поле называется "time" и имеет тип Duration
        wait_time = goal_handle.request.time
        total_wait = wait_time.sec + wait_time.nanosec / 1e9
        self.get_logger().info(f"Goal wait time: {total_wait:.2f} seconds")

        feedback_msg = Wait.Feedback()  # Объект обратной связи (возможно, пустой)
        result_msg = Wait.Result()

        # Имитируем ожидание: каждую секунду логируем оставшееся время
        start_time = time.time()
        elapsed = 0.0
        remaining = total_wait - elapsed
        while remaining > 0:
            self.get_logger().info(f"count_action Feedback: remaining {remaining:.2f} seconds")
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)
            elapsed = time.time() - start_time
            remaining = total_wait - elapsed

        goal_handle.succeed()
        result = Duration()
        result.sec = int(total_wait)
        result.nanosec = int((total_wait - int(total_wait)) * 1e9)
        # Используем правильное имя поля: 'remaining'
        result_msg.remaining = result
        self.get_logger().info(f"count_action Returning result: waited {total_wait:.2f} seconds")
        return result_msg


def main(args=None):
    rclpy.init(args=args)
    action_server = MultiActionServer()
    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        action_server.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
