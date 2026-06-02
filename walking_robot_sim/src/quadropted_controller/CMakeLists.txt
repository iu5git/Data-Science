cmake_minimum_required(VERSION 3.5)
project(quadropted_controller)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python package with entry points
ament_python_install_package(
  ${PROJECT_NAME}
  PACKAGE_DIR scripts
)

# Install executables
install(PROGRAMS
  scripts/robot_controller_gazebo.py
  scripts/QuadrupedOdometryNode.py
  scripts/cmd_vel_pub.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()