cmake_minimum_required(VERSION 3.8)
project(hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interfaces REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(robot_introspection src/robot_introspection.cpp)
add_executable(light_sensors src/light_sensors.cpp)
add_executable(lidar_sensor  src/lidar_sensor.cpp)

ament_target_dependencies(robot_introspection rclcpp std_msgs interfaces)
ament_target_dependencies(light_sensors rclcpp std_msgs interfaces)
ament_target_dependencies(lidar_sensor  rclcpp std_msgs interfaces sensor_msgs)

target_include_directories(robot_introspection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(robot_introspection PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS
  robot_introspection
  light_sensors
  lidar_sensor
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
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
