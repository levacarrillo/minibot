cmake_minimum_required(VERSION 3.8)
project(hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(rviz2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(random_numbers REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(lidar_simulator src/lidar_simulator.cpp)
ament_target_dependencies(lidar_simulator rclcpp sensor_msgs interfaces tf2 tf2_ros geometry_msgs random_numbers)

add_executable(light_sensors_simulator src/light_sensors_simulator.cpp)
ament_target_dependencies(light_sensors_simulator rclcpp interfaces visualization_msgs tf2_ros geometry_msgs)

add_executable(robot_introspection src/robot_introspection.cpp)
ament_target_dependencies(robot_introspection rclcpp interfaces)

add_executable(spot_light_marker src/spot_light_marker.cpp)
ament_target_dependencies(spot_light_marker rclcpp visualization_msgs geometry_msgs tf2_ros)

add_executable(static_tf_map_to_odom src/static_tf_map_to_odom.cpp)
ament_target_dependencies(static_tf_map_to_odom rclcpp geometry_msgs tf2 tf2_ros)

install(TARGETS
  lidar_simulator
  light_sensors_simulator
  robot_introspection
  spot_light_marker
  static_tf_map_to_odom
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
