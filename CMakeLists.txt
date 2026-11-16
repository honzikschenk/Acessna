cmake_minimum_required(VERSION 3.28)
project(acessna LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# JSBSim
include(FetchContent)
set(BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_MODULE OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(jsbsim
  GIT_REPOSITORY https://github.com/JSBSim-Team/jsbsim.git
  GIT_TAG        v1.3.3
  GIT_SHALLOW    TRUE
  EXCLUDE_FROM_ALL
  SYSTEM)
FetchContent_MakeAvailable(jsbsim)

# Acessna
find_package(Threads REQUIRED)  # Monte Carlo runner
add_library(acessna
  src/telemetry.cpp
  src/sim/sim.cpp
  src/sim/harness.cpp
  src/autopilot/autopilot.cpp
  src/autopilot/mode_manager.cpp
  src/autopilot/modes.cpp
  src/guidance/glideslope.cpp
  src/guidance/l1.cpp
  src/guidance/route.cpp
  src/scenario/kroa.cpp
  src/scenario/metrics.cpp
  src/control/pid.cpp
  src/control/filters.cpp
  src/control/flight_path_controller.cpp
  src/control/tecs.cpp
  src/control/lateral_controller.cpp
  src/control/attitude_controller.cpp
  src/control/rate_controller.cpp)
target_include_directories(acessna PUBLIC include)
target_link_libraries(acessna PUBLIC libJSBSim Threads::Threads)
target_compile_definitions(acessna PRIVATE
  ACESSNA_JSBSIM_ROOT="${jsbsim_SOURCE_DIR}")
target_compile_options(acessna PRIVATE -Wall -Wextra)

# Tests (GoogleTest)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_Declare(googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        v1.15.2
  GIT_SHALLOW    TRUE
  EXCLUDE_FROM_ALL
  SYSTEM)
FetchContent_MakeAvailable(googletest)

enable_testing()
include(GoogleTest)
add_executable(acessna_tests
  tests/test_harness.cpp
  tests/test_infrastructure.cpp
  tests/test_route.cpp
  tests/test_autopilot.cpp)
target_link_libraries(acessna_tests PRIVATE acessna GTest::gtest_main)
gtest_discover_tests(acessna_tests)

# Executable
add_executable(acessna_run src/main.cpp)
target_link_libraries(acessna_run PRIVATE acessna)

