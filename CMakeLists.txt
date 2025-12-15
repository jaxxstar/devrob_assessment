cmake_minimum_required(VERSION 3.8)
project(devrob_assessment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(waypoint_mover src/waypoint_mover.cpp)
ament_target_dependencies(waypoint_mover rclcpp moveit_ros_planning_interface geometry_msgs)

install(TARGETS
  waypoint_mover
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config 
  DESTINATION share/${PROJECT_NAME}
)

ament_package() 