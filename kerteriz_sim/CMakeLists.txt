cmake_minimum_required(VERSION 3.16)
project(kerteriz_sim VERSION 0.1.0 LANGUAGES CXX)

# Deterministik yorunge/sensor ureteci (PHASE0.md S9).
# ROS'a bagimli DEGILDIR — sentetik dogrulama ROS'suz kosabilmelidir (R6).

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Testler GENEL BUILD_TESTING'e bagli DEGILDIR — bilerek.
# BUILD_TESTING global bir addir ve ust projeler veya bagimliliklar tarafindan
# degistirilebilir; nitekim kerteriz_core bir donem onu cache'e zorluyordu ve
# sizdiriyordu. Testlerin sessizce kapanmamasi icin kendi bayragimiz kullanilir.
option(KERTERIZ_SIM_BUILD_TESTING "kerteriz_sim testlerini derle" ON)

find_package(kerteriz_core QUIET)
if(NOT kerteriz_core_FOUND)
  # Bagimsiz build: kardes dizinden dogrudan al.
  #
  # core'un KENDI testleri burada KAPATILIR. Aksi halde EXCLUDE_FROM_ALL yuzunden
  # derlenmeyen test hedefleri ctest'e kaydolur ve "Not Run" olarak DUSER.
  # core'un testleri core'un kendi build'inde kosar; sim'in isi degil.
  # Genel BUILD_TESTING degil, core'un kendi bayragi kullanilir (CMP0077 NEW
  # sayesinde normal degisken option()'i ezer).
  set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
  set(KERTERIZ_BUILD_TESTING OFF)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../kerteriz_core
                   ${CMAKE_CURRENT_BINARY_DIR}/kerteriz_core EXCLUDE_FROM_ALL)
endif()

add_library(kerteriz_sim INTERFACE)
add_library(kerteriz::sim ALIAS kerteriz_sim)
target_include_directories(kerteriz_sim INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(kerteriz_sim INTERFACE kerteriz::core)
target_compile_features(kerteriz_sim INTERFACE cxx_std_17)

install(DIRECTORY include/ DESTINATION include)

# -----------------------------------------------------------------------------
# Testler
# -----------------------------------------------------------------------------
if(KERTERIZ_SIM_BUILD_TESTING)
  enable_testing()

  # Bagimsiz yolda kerteriz_core zaten GoogleTest'i getirmis olabilir; getirdiyse
  # yeniden indirilmez. Aksi halde once sistemde aranir, yoksa pinli surum alinir.
  if(NOT TARGET GTest::gtest_main)
    find_package(GTest QUIET)
  endif()
  if(NOT TARGET GTest::gtest_main)
    include(FetchContent)
    set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
    FetchContent_Declare(googletest
      GIT_REPOSITORY https://github.com/google/googletest.git
      GIT_TAG        v1.14.0
      GIT_SHALLOW    TRUE)
    set(gtest_force_shared_crt ON)
    set(INSTALL_GTEST OFF)
    FetchContent_MakeAvailable(googletest)
  endif()

  add_subdirectory(test)
endif()
