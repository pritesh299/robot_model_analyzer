cmake_minimum_required(VERSION 3.22)
project(robot_model_analyzer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(pinocchio REQUIRED)

# Build and install the executable.
# NOTE: You must use `target_link_libraries` to get collision checking capabilities.
add_executable(robot_model_analyzer src/robot_model_analyzer.cpp)
ament_target_dependencies(robot_model_analyzer ament_index_cpp rclcpp std_msgs Eigen3 eigen3_cmake_module)
target_link_libraries(robot_model_analyzer pinocchio::pinocchio)

install(
  TARGETS
  robot_model_analyzer
  DESTINATION lib/robot_model_analyzer
)


ament_package()
