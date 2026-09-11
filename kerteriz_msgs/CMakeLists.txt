cmake_minimum_required(VERSION 3.16)
project(kerteriz_msgs VERSION 0.1.0)

find_package(ament_cmake REQUIRED)

# Mesaj tanimlari Faz 1+'da gelir:
#   EstimatorDiagnostics (ADR-8), ProcessingResult/RejectReason (ADR-19),
#   ProtectionLevel (Faz 4), SensorHealth (ADR-18).
# Ilk .msg eklendiginde rosidl_generate_interfaces devreye alinir.
file(GLOB KERTERIZ_MSG_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*.msg)
if(KERTERIZ_MSG_FILES)
  find_package(rosidl_default_generators REQUIRED)
  find_package(std_msgs REQUIRED)
  rosidl_generate_interfaces(${PROJECT_NAME} ${KERTERIZ_MSG_FILES}
                             DEPENDENCIES std_msgs)
  ament_export_dependencies(rosidl_default_runtime)
else()
  message(STATUS "kerteriz_msgs: henuz mesaj tanimi yok — iskelet derleniyor")
endif()

ament_package()
