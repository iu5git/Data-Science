"""
Запуск Gazebo симуляции с C++ контроллером (quadropted_controller_cpp).
Аналог launch_python.launch.py, но использует C++ узлы.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    ld = LaunchDescription()
    package_name = "gazebo_sim"
    pkg_path = get_package_share_directory(package_name)

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Использовать симуляционное время",
        )
    )
    ld.add_action(SetParameter(name="use_sim_time", value=use_sim_time))

    camera_fps = LaunchConfiguration("camera_fps", default="10")
    ld.add_action(
        DeclareLaunchArgument(
            name="camera_fps",
            default_value="10",
            description="Camera update rate (FPS)",
        )
    )

    world_file = os.path.join(pkg_path, "world", "cafe.world")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_args": ["-r -v4 ", world_file],
            "on_exit_shutdown": "true",
        }.items(),
    )
    ld.add_action(gazebo)

    pause = ExecuteProcess(cmd=["sleep", "6"], output="screen")
    ld.add_action(pause)

    # Вместо gazebo_multi_nav2_world.launch.py запускаем C++ версию
    multi_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, "launch", "gazebo_multi_nav2_cpp.launch.py")
        ),
        launch_arguments={
            "camera_fps": camera_fps,
        }.items(),
    )

    ld.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=pause, on_exit=[multi_nav2_launch]
            )
        )
    )

    experiment_logger = Node(
        package='gazebo_sim',
        executable='experiment_logger.py',
        namespace='robot1',
        name='experiment_logger',
        output='screen',
        parameters=[{
            'odom_topic': '/robot1/odom',
            'output_dir': '/tmp/experiments',
        }],
    )
    ld.add_action(experiment_logger)

    return ld
