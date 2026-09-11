cmake_minimum_required(VERSION 3.16)
project(kerteriz_sim VERSION 0.1.0 LANGUAGES CXX)

# Deterministik yorunge/sensor ureteci (PHASE0.md S9).
# ROS'a bagimli DEGILDIR — sentetik dogrulama ROS'suz kosabilmelidir (R6).

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(kerteriz_core QUIET)
if(NOT kerteriz_core_FOUND)
  # Bagimsiz build: kardes dizinden dogrudan al.
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../kerteriz_core
                   ${CMAKE_CURRENT_BINARY_DIR}/kerteriz_core EXCLUDE_FROM_ALL)
endif()

add_library(kerteriz_sim INTERFACE)
add_library(kerteriz::sim ALIAS kerteriz_sim)
target_include_directories(kerteriz_sim INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(kerteriz_sim INTERFACE kerteriz::core)
target_compile_features(kerteriz_sim INTERFACE cxx_std_17)

install(DIRECTORY include/ DESTINATION include)
