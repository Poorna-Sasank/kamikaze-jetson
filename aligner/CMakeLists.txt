cmake_minimum_required(VERSION 3.8)
project(aligner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

# Executables
add_executable(aligner align/yawAlign.cpp)
add_executable(roacher approach/roacher.cpp)
add_executable(loitering  loiter/loiter_control.cpp)
add_executable(learning_control learning_control/descent_line.cpp)
add_executable(goto_pos goto_pos/goto_pos.cpp)

# Dependencies
set(DEPENDENCIES
  rclcpp
  px4_ros2_cpp
  Eigen3
  geometry_msgs
  geographic_msgs
  tf2_ros
  tf2
)

ament_target_dependencies(aligner ${DEPENDENCIES})
ament_target_dependencies(roacher ${DEPENDENCIES})
ament_target_dependencies(loitering ${DEPENDENCIES})
ament_target_dependencies(learning_control ${DEPENDENCIES})
ament_target_dependencies(goto_pos ${DEPENDENCIES})

# C++ standard
target_compile_features(aligner PUBLIC cxx_std_17)
target_compile_features(roacher PUBLIC cxx_std_17)
target_compile_features(loitering PUBLIC cxx_std_17)
target_compile_features(learning_control PUBLIC cxx_std_17)
target_compile_features(goto_pos PUBLIC cxx_std_17)

# Install
install(TARGETS aligner roacher loitering learning_control goto_pos
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch cfg
  DESTINATION share/${PROJECT_NAME}/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
