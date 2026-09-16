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

add_library(kerteriz_bringup_adapters INTERFACE)
add_library(kerteriz::bringup_adapters ALIAS kerteriz_bringup_adapters)
target_include_directories(kerteriz_bringup_adapters INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(kerteriz_bringup_adapters INTERFACE kerteriz::core)
target_compile_features(kerteriz_bringup_adapters INTERFACE cxx_std_17)

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

  ament_add_gtest(test_dataset_adapters test/test_dataset_adapters.cpp)
  target_link_libraries(test_dataset_adapters kerteriz_bringup_adapters)
  target_compile_definitions(test_dataset_adapters
    PRIVATE KERTERIZ_FIXTURE_DIR="${KERTERIZ_FIXTURE_DIR}")
endif()

ament_package()
