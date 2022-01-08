cmake_minimum_required(VERSION 3.8)
project(ros2-lifecycle)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  std_msgs
)
add_executable(lifecycle_talker
  src/lifecycle_talker_node.cpp
  src/lifecycle_talker.cpp
)
ament_target_dependencies(lifecycle_talker ${dependencies})

install(TARGETS lifecycle_talker
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(normal_listener
  src/normal_listener_node.cpp
  src/normal_listener.cpp
)
ament_target_dependencies(normal_listener ${dependencies})

install(TARGETS normal_listener
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
