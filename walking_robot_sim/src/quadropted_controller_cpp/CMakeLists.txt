cmake_minimum_required(VERSION 3.8)
project(quadropted_controller_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(quadropted_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# ════════════════════════════════════════════════════════════
# Библиотека
# ════════════════════════════════════════════════════════════

add_library(${PROJECT_NAME} SHARED
  # Utils
  src/utils/math_utils.cpp
  src/utils/rotation_matrices.cpp
  src/utils/homogeneous_transforms.cpp
  src/utils/message_builders.cpp
  # Kinematics
  src/kinematics/forward_kinematics.cpp
  src/kinematics/inverse_kinematics.cpp
  # Odometry
  src/odometry/odometry_state.cpp
  src/odometry/odometry_update.cpp
  # States
  src/states/state_command.cpp
  # Controllers
  src/controllers/pid_controller.cpp
  src/controllers/gait_controller.cpp
  src/controllers/rest_controller.cpp
  src/controllers/stand_controller.cpp
  src/controllers/trot_stance.cpp
  src/controllers/trot_swing.cpp
  src/controllers/trot_gait.cpp
  src/controllers/crawl_stance.cpp
  src/controllers/crawl_swing.cpp
  src/controllers/crawl_gait.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp geometry_msgs nav_msgs sensor_msgs tf2_ros tf2 quadropted_msgs Eigen3)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

# ════════════════════════════════════════════════════════════
# Исполняемые файлы (ROS 2 узлы)
# ════════════════════════════════════════════════════════════

add_executable(odometry_node src/nodes/odometry_node.cpp)
ament_target_dependencies(odometry_node visualization_msgs rclcpp nav_msgs sensor_msgs geometry_msgs tf2_ros quadropted_msgs Eigen3)
target_link_libraries(odometry_node ${PROJECT_NAME})

add_executable(robot_controller_node src/nodes/robot_controller_node.cpp)
ament_target_dependencies(robot_controller_node rclcpp std_msgs sensor_msgs tf2_ros quadropted_msgs Eigen3)
target_link_libraries(robot_controller_node ${PROJECT_NAME})

add_executable(cmd_vel_pub src/nodes/cmd_vel_pub.cpp)
ament_target_dependencies(cmd_vel_pub rclcpp geometry_msgs quadropted_msgs)
target_link_libraries(cmd_vel_pub ${PROJECT_NAME})

install(TARGETS odometry_node robot_controller_node cmd_vel_pub DESTINATION lib/${PROJECT_NAME})

# ════════════════════════════════════════════════════════════
# Тесты
# ════════════════════════════════════════════════════════════

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(test_rotation_matrices test/test_rotation_matrices.cpp)
  target_link_libraries(test_rotation_matrices ${PROJECT_NAME})

  ament_add_gmock(test_homogeneous_transforms test/test_homogeneous_transforms.cpp)
  target_link_libraries(test_homogeneous_transforms ${PROJECT_NAME})

  ament_add_gmock(test_fk test/test_fk.cpp)
  target_link_libraries(test_fk ${PROJECT_NAME})

  ament_add_gmock(test_ik test/test_ik.cpp)
  target_link_libraries(test_ik ${PROJECT_NAME})

  ament_add_gmock(test_odometry test/test_odometry.cpp)
  target_link_libraries(test_odometry ${PROJECT_NAME})

  ament_add_gmock(test_pid test/test_pid.cpp)
  target_link_libraries(test_pid ${PROJECT_NAME})

  ament_add_gmock(test_gait test/test_gait.cpp)
  target_link_libraries(test_gait ${PROJECT_NAME})

  ament_add_gmock(test_message_builders test/test_message_builders.cpp)
  ament_add_gmock(test_cross_validation test/test_cross_validation.cpp)
  target_link_libraries(test_cross_validation ${PROJECT_NAME})
  target_link_libraries(test_message_builders ${PROJECT_NAME})

  # Base link roll tests — критические тесты для проверки крена
  ament_add_gmock(test_base_link_roll test/test_base_link_roll.cpp)
  target_link_libraries(test_base_link_roll ${PROJECT_NAME})

  # IK with roll tests — проверка влияния крена на углы суставов
  ament_add_gmock(test_ik_with_roll test/test_ik_with_roll.cpp)
  target_link_libraries(test_ik_with_roll ${PROJECT_NAME})

  # step_trot cross-validation tests
  ament_add_gmock(test_step_trot test/test_step_trot.cpp)
  target_link_libraries(test_step_trot ${PROJECT_NAME})
endif()

# ════════════════════════════════════════════════════════════
# Benchmark (standalone, no ROS dependencies)
# ════════════════════════════════════════════════════════════

add_executable(benchmark benchmark/benchmark.cpp)
target_link_libraries(benchmark ${PROJECT_NAME})

# ════════════════════════════════════════════════════════════
# Установка
# ════════════════════════════════════════════════════════════

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(visualization_msgs rclcpp geometry_msgs nav_msgs sensor_msgs tf2_ros tf2 quadropted_msgs Eigen3)
ament_package()
