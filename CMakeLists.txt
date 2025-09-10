cmake_minimum_required(VERSION 3.8)
project(px4_test_scripts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Require ament_cmake
find_package(ament_cmake REQUIRED)

# ROS2 packages
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Eigen
find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ----------------- Executables -----------------

add_executable(ground_controller src/ground_controller.cpp)
ament_target_dependencies(ground_controller
  rclcpp
  geometry_msgs
  mavros_msgs
  geographic_msgs
  sensor_msgs
  std_msgs
)
target_include_directories(ground_controller PUBLIC ${EIGEN3_INCLUDE_DIRS})

add_executable(drone_mc src/drone_mc.cpp)
ament_target_dependencies(drone_mc
  rclcpp
  geometry_msgs
  mavros_msgs
  geographic_msgs
  std_msgs
)
target_include_directories(drone_mc PUBLIC ${EIGEN3_INCLUDE_DIRS})

# ----------------- Installation -----------------
install(TARGETS
  ground_controller
  drone_mc
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# ----------------- Testing -----------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
