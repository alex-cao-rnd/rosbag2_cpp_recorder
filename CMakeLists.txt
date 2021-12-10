cmake_minimum_required(VERSION 3.5)
project(bag_recorder)

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
find_package(rosbag2_cpp REQUIRED)
find_package(example_interfaces REQUIRED)

find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ulisse_msgs REQUIRED)

include_directories(
  include
)


add_library(RosbagRecorder src/bag_recorder/rosbag_recorder.cpp)
ament_target_dependencies(RosbagRecorder rclcpp rosbag2_cpp)

add_executable(simple_bag_recorder src/simple_bag_recorder.cpp)
ament_target_dependencies(simple_bag_recorder rclcpp rosbag2_cpp)

add_executable(bag_recorder_wrapper src/bag_recorder_wrapper.cpp)
ament_target_dependencies(bag_recorder_wrapper rclcpp rosbag2_cpp std_srvs ulisse_msgs)

install(TARGETS
  simple_bag_recorder
  bag_recorder_wrapper
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
