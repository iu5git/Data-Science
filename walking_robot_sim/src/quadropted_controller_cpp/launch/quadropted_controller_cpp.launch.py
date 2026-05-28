from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="quadropted_controller_cpp",
                executable="odometry_node",
                name="dog_odometry_cpp",
                namespace="robot1",
                parameters=[{"verbose": False, "publish_rate": 50, "is_gazebo": True, "enable_odom_tf": True}],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            ),
            Node(
                package="quadropted_controller_cpp",
                executable="robot_controller_node",
                name="robot_controller_cpp",
                namespace="robot1",
                parameters=[{"verbose": False}],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            ),
        ]
    )
