cmake_minimum_required(VERSION 3.8)
project(rviz_waypoint_tool)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(pluginlib REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(${PROJECT_NAME} SHARED
  src/custom_goal_tool.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  rviz_common
  rviz_default_plugins
  pluginlib
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp geometry_msgs rviz_common rviz_default_plugins pluginlib)
ament_package()