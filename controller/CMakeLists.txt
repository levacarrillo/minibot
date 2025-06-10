cmake_minimum_required(VERSION 3.8)
project(controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(line_follower src/line_follower.cpp)
ament_target_dependencies(line_follower rclcpp geometry_msgs)

add_executable(move_in_circles src/move_in_circles.cpp)
ament_target_dependencies(move_in_circles rclcpp geometry_msgs)

add_executable(pose_commander src/pose_commander.cpp)
ament_target_dependencies(pose_commander rclcpp geometry_msgs interfaces rclcpp_action)


install(TARGETS
  line_follower
  move_in_circles
  pose_commander
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
