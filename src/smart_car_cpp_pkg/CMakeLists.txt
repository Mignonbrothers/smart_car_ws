cmake_minimum_required(VERSION 3.8)
project(smart_car_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

set(COMMON_DEPS
  rclcpp
  geometry_msgs
)

add_executable(go_to_pose src/pi4/go_to_pose.cpp)
target_include_directories(go_to_pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(go_to_pose PUBLIC cxx_std_17)
ament_target_dependencies(
  go_to_pose
  ${COMMON_DEPS}
  rclcpp_action
  nav2_msgs
)

add_executable(person_follower
  src/pi4/person_follower.cpp
  src/pi4/kalman_filter.cpp
)
target_include_directories(person_follower PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(person_follower PUBLIC cxx_std_17)
ament_target_dependencies(
  person_follower
  ${COMMON_DEPS}
  std_msgs
  sensor_msgs
)

install(TARGETS
  go_to_pose
  person_follower
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
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
