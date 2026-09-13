cmake_minimum_required(VERSION 3.16)
project(kerteriz_core VERSION 0.1.0 LANGUAGES CXX)

# =============================================================================
# kerteriz_core — saf C++17 kestirim çekirdeği.
#
# R1 (CLAUDE.md) / ADR-1: bu hedef ROS'a bağımlı DEĞİLDİR.
#   - ROS build sistemi kullanılmaz; düz CMake ile derlenir
#   - Hiçbir ROS istemci kütüphanesi veya arayüz paketi aranmaz
#   - ROS kurulu olmayan bir makinede tek başına derlenir ve test edilir
#
# NOT: bu dosyadaki yorumlar yasaklı ROS sembollerini LİTERAL olarak içermez;
# böylece denetim ① (CLAUDE.md §6.1) yorum ayıklamadan da temiz çalışır.
#
# İki yollu build:
#   1) bağımsız : cmake -S kerteriz_core -B build/core
#   2) colcon   : package.xml içinde <build_type>cmake</build_type>
# =============================================================================

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# ADR-12 / R8: Humble + GCC 11 hedefi. C++20 kullanılmaz.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# -----------------------------------------------------------------------------
# Bağımlılık seçenekleri — CACHE'e YAZILMAZ
#
# Daha önce bu seçenekler `set(X OFF CACHE BOOL "" FORCE)` ile ayarlanıyordu.
# BUILD_TESTING, BUILD_EXAMPLES gibi GENEL adlar global cache'e zorlandığı için
# dışarı sızıyordu:
#   - `cmake -DBUILD_TESTING=ON` diyen kullanıcının seçimi sessizce OFF'a
#     çevriliyordu;
#   - kerteriz_core'u `add_subdirectory` ile ekleyen bir üst proje
#     (kerteriz_sim'in bağımsız yolu) kendi BUILD_TESTING değerini kaybediyordu.
#
# Çözüm: normal (dizin kapsamlı) değişken kullanmak. Cache'e dokunulmaz; değer
# yalnızca bu dizin ve altındaki FetchContent alt projeleri için geçerlidir.
# CMP0077 NEW, alt projelerdeki `option()` çağrılarının var olan normal
# değişkeni ezmemesini sağlar — alt projenin kendi cmake_minimum_required'ı
# daha eski olsa bile.
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

option(KERTERIZ_BUILD_TESTING "kerteriz_core testlerini derle" ON)

# -----------------------------------------------------------------------------
# S2 · Bağımlılıklar — sürümler PİNLİ (PHASE0.md S2)
# -----------------------------------------------------------------------------

# --- Eigen 3.4 -----------------------------------------------------------
# Ubuntu 22.04'te libeigen3-dev ile sistemde vardır. Bulunamazsa (temiz
# konteyner, Windows) pinli sürüm indirilir; hedef ortam davranışı değişmez.
find_package(Eigen3 3.4 QUIET NO_MODULE)
if(Eigen3_FOUND)
  message(STATUS "kerteriz_core: sistem Eigen3 ${Eigen3_VERSION} kullanılıyor")
else()
  message(STATUS "kerteriz_core: sistemde Eigen3 3.4 yok — FetchContent ile 3.4.0 indiriliyor")
  FetchContent_Declare(eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
    GIT_SHALLOW    TRUE)
  set(EIGEN_BUILD_DOC OFF)
  set(EIGEN_BUILD_TESTING OFF)
  set(EIGEN_BUILD_PKGCONFIG OFF)
  set(BUILD_TESTING OFF)
  FetchContent_MakeAvailable(eigen)
endif()

# --- manif 0.0.5 ---------------------------------------------------------
# CONVENTIONS §2/§3: Hamilton kuaterniyonu ve sağ pertürbasyon manif ile uyumlu.
# GIT_TAG sabittir — master KULLANILMAZ (PHASE0.md S2).
FetchContent_Declare(manif
  GIT_REPOSITORY https://github.com/artivis/manif.git
  GIT_TAG        0.0.5
  GIT_SHALLOW    TRUE)
set(BUILD_TESTING OFF)
set(BUILD_EXAMPLES OFF)
set(BUILD_BENCHMARKING OFF)
set(BUILD_PYTHON_BINDINGS OFF)
FetchContent_MakeAvailable(manif)

# manif header-only'dir. Hedef adı sürümden sürüme değişebildiği için
# önce alias aranır, yoksa include dizini doğrudan kullanılır.
if(TARGET MANIF::manif)
  set(KERTERIZ_MANIF_LINK MANIF::manif)
elseif(TARGET manif)
  set(KERTERIZ_MANIF_LINK manif)
else()
  add_library(kerteriz_manif_headers INTERFACE)
  target_include_directories(kerteriz_manif_headers
    INTERFACE $<BUILD_INTERFACE:${manif_SOURCE_DIR}/include>)
  set(KERTERIZ_MANIF_LINK kerteriz_manif_headers)
endif()
message(STATUS "kerteriz_core: manif hedefi = ${KERTERIZ_MANIF_LINK}")

# -----------------------------------------------------------------------------
# Hedef: kerteriz::core
# -----------------------------------------------------------------------------
# Şu an header-only (INTERFACE). İlk .cpp geldiğinde STATIC'e çevrilecek;
# tüketici tarafı `kerteriz::core` adını kullanmaya devam eder.
add_library(kerteriz_core INTERFACE)
add_library(kerteriz::core ALIAS kerteriz_core)

# Kurulu export'taki ad build agacindaki ALIAS ile AYNI olmalidir.
# EXPORT_NAME olmadan install(EXPORT ... NAMESPACE kerteriz::) hedefi
# `kerteriz::kerteriz_core` diye disa yazar; tuketiciler ise `kerteriz::core`
# bekler. Fark yalnizca hedefi GERCEKTEN linkleyen ilk tuketicide patlar —
# find_package'in tek basina gecmesi bunu gizler.
set_target_properties(kerteriz_core PROPERTIES EXPORT_NAME core)

target_include_directories(kerteriz_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_compile_features(kerteriz_core INTERFACE cxx_std_17)

if(Eigen3_FOUND)
  target_link_libraries(kerteriz_core INTERFACE Eigen3::Eigen ${KERTERIZ_MANIF_LINK})
else()
  target_link_libraries(kerteriz_core INTERFACE eigen ${KERTERIZ_MANIF_LINK})
endif()

# -----------------------------------------------------------------------------
# Testler — GoogleTest 1.14.0 (C++17 uyumlu son sürümlerden)
# -----------------------------------------------------------------------------
if(KERTERIZ_BUILD_TESTING)
  enable_testing()
  FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
    GIT_SHALLOW    TRUE)
  set(gtest_force_shared_crt ON)
  set(INSTALL_GTEST OFF)
  set(BUILD_GMOCK ON)
  FetchContent_MakeAvailable(googletest)
  add_subdirectory(test)
endif()

# -----------------------------------------------------------------------------
# Kurulum ve export — kerteriz_ros bunu find_package ile bulur
# -----------------------------------------------------------------------------
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS kerteriz_core EXPORT kerteriz_coreTargets)
install(EXPORT kerteriz_coreTargets
  FILE        kerteriz_coreTargets.cmake
  NAMESPACE   kerteriz::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/kerteriz_core)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/kerteriz_coreConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/kerteriz_coreConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/kerteriz_core)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/kerteriz_coreConfigVersion.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/kerteriz_coreConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/kerteriz_coreConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/kerteriz_core)
