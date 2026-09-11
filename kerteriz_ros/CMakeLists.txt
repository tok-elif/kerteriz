cmake_minimum_required(VERSION 3.16)
project(kerteriz_ros VERSION 0.1.0 LANGUAGES CXX)

# ROS 2 adaptor katmani. Is mantigi BURAYA YAZILMAZ (CLAUDE.md §4).
# Kestirim matematigi kerteriz_core'dadir.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(kerteriz_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Kaynaklar Faz 1'de gelir (EstimatorNode, conversions).
# Iskelet asamasinda yalnizca bagimliliklar dogrulanir.

ament_export_dependencies(kerteriz_core rclcpp rclcpp_lifecycle rclcpp_components
                          nav_msgs sensor_msgs geometry_msgs diagnostic_msgs tf2_ros)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
