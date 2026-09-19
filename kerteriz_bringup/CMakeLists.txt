cmake_minimum_required(VERSION 3.16)
project(kerteriz_bringup VERSION 0.1.0 LANGUAGES CXX)

# Launch/YAML ve VERI SETI ADAPTORLERI (CLAUDE.md §4).
#
# Adaptorler ROS'a bagli DEGILDIR: duz C++17 + Eigen + kerteriz_core. ROS
# burada yalnizca paketleme/kurulum icindir. Ayristirma mantiginin ROS'suz
# kalmasi, formatlarin birim testlerinin gercek veri seti ve ROS calisma zamani
# olmadan kosabilmesini saglar.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(kerteriz_core REQUIRED)

# ROS bagimliliklari YALNIZCA harici taban cizgisi kosturmasi icindir
# (robot_localization'a veri seti yayinlamak ve ciktisini kaydetmek).
# Ayristiricilar ve Kerteriz kosucusu ROS'suzdur.
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)

add_library(kerteriz_bringup_adapters INTERFACE)
add_library(kerteriz::bringup_adapters ALIAS kerteriz_bringup_adapters)
target_include_directories(kerteriz_bringup_adapters INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(kerteriz_bringup_adapters INTERFACE kerteriz::core)
target_compile_features(kerteriz_bringup_adapters INTERFACE cxx_std_17)

# Gercek veri seti kosucusu (Checkpoint B). ROS'a BAGLI DEGILDIR: veri seti
# cevrimdisi okunur, uretim zinciri dogrudan cagrilir.
add_executable(kerteriz_kitti_runner src/kitti_runner_main.cpp)
target_link_libraries(kerteriz_kitti_runner kerteriz_bringup_adapters)

add_executable(kerteriz_ate src/ate_main.cpp)
target_link_libraries(kerteriz_ate kerteriz_bringup_adapters)

# --- HARICI taban cizgisi kosturmasi (Kerteriz filtresinin parcasi DEGIL) ---
add_executable(kerteriz_kitti_baseline_publisher src/kitti_baseline_publisher_main.cpp)
target_link_libraries(kerteriz_kitti_baseline_publisher kerteriz_bringup_adapters)
ament_target_dependencies(kerteriz_kitti_baseline_publisher
  rclcpp sensor_msgs nav_msgs rosgraph_msgs)

add_executable(kerteriz_baseline_recorder src/baseline_recorder_main.cpp)
ament_target_dependencies(kerteriz_baseline_recorder rclcpp nav_msgs)

install(TARGETS
  kerteriz_kitti_runner
  kerteriz_ate
  kerteriz_kitti_baseline_publisher
  kerteriz_baseline_recorder
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# Testler GENEL BUILD_TESTING'e bagli DEGILDIR — bilerek (S9'daki sizinti).
# BUILD_TESTING ust projeler tarafindan degistirilebilir; testlerin sessizce
# kapanmamasi icin kendi bayragimiz kullanilir.
option(KERTERIZ_BRINGUP_BUILD_TESTING "kerteriz_bringup testlerini derle" ON)

if(KERTERIZ_BRINGUP_BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Fixture'lar SENTETIKTIR (test/fixtures/OKUBENI.md). Gercek veri seti
  # depoya KONMAZ ve CI indirmez.
  set(KERTERIZ_FIXTURE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")

  ament_add_gtest(test_geodetic test/test_geodetic.cpp)
  target_link_libraries(test_geodetic kerteriz_bringup_adapters)

  ament_add_gtest(test_ate test/test_ate.cpp)
  target_link_libraries(test_ate kerteriz_bringup_adapters)

  # F2.4-C — deterministik GNSS konum seyreltme politikasi (saf, paylasimli).
  ament_add_gtest(test_gnss_sampling test/test_gnss_sampling.cpp)
  target_link_libraries(test_gnss_sampling kerteriz_bringup_adapters)

  # F2.4 — withheld deney sozlesmesinin CLI kapisi (saf, veri seti gerektirmez).
  ament_add_gtest(test_withheld_contract test/test_withheld_contract.cpp)
  target_link_libraries(test_withheld_contract kerteriz_bringup_adapters)

  ament_add_gtest(test_kitti_runner test/test_kitti_runner.cpp)
  target_link_libraries(test_kitti_runner kerteriz_bringup_adapters)
  target_compile_definitions(test_kitti_runner
    PRIVATE KERTERIZ_FIXTURE_DIR="${KERTERIZ_FIXTURE_DIR}")

  ament_add_gtest(test_dataset_adapters test/test_dataset_adapters.cpp)
  target_link_libraries(test_dataset_adapters kerteriz_bringup_adapters)
  target_compile_definitions(test_dataset_adapters
    PRIVATE KERTERIZ_FIXTURE_DIR="${KERTERIZ_FIXTURE_DIR}")
endif()

ament_package()
