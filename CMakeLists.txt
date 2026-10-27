cmake_minimum_required(VERSION 3.24)
project(haul_vcu VERSION 0.1.0 DESCRIPTION "Vehicle control unit for an autonomous haul truck" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(HAUL_BUILD_TESTS "Build the unit and scenario tests" ON)
option(HAUL_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
set(HAUL_SANITIZE "" CACHE STRING "Sanitizers to enable: address,undefined | thread | empty")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

include(cmake/Warnings.cmake)
include(cmake/Dependencies.cmake)

# ---- core library: everything except main() --------------------------------------------
add_library(haul
  src/can/dbc.cpp
  src/can/e2e.cpp
  src/can/candump.cpp
  src/can/udp_bus.cpp
  src/can/socketcan_bus.cpp
  src/vcu/calibration.cpp
  src/vcu/faults.cpp
  src/vcu/estimator.cpp
  src/vcu/speed_controller.cpp
  src/vcu/monitors.cpp
  src/vcu/state_machine.cpp
  src/vcu/vcu.cpp
  src/sim/truck_model.cpp
  src/sim/truck_sim.cpp
  src/sim/scenario.cpp
  src/sim/sil_runner.cpp
  src/sim/report.cpp
  src/rt/vcu_node.cpp
  src/core/rt_thread.cpp
)
target_include_directories(haul PUBLIC include)
target_link_libraries(haul PUBLIC nlohmann_json::nlohmann_json Threads::Threads)
target_compile_definitions(haul PUBLIC HAUL_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
haul_set_warnings(haul)

# ---- programs --------------------------------------------------------------------------
foreach(app haul-sim haul-vcu haul-truck haul-can haul-bench)
  string(REPLACE "-" "_" src ${app})
  add_executable(${app} apps/${src}.cpp)
  target_link_libraries(${app} PRIVATE haul)
  haul_set_warnings(${app})
endforeach()

# ---- tests -----------------------------------------------------------------------------
if(HAUL_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
