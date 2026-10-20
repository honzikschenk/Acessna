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
add_library(acessna
  src/sim/sim.cpp
  src/sim/harness.cpp
  src/control/flight_controller.cpp)
target_include_directories(acessna PUBLIC include)
target_link_libraries(acessna PUBLIC libJSBSim)
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
add_executable(acessna_tests tests/test_harness.cpp)
target_link_libraries(acessna_tests PRIVATE acessna GTest::gtest_main)
gtest_discover_tests(acessna_tests)

# Executable
add_executable(acessna_run src/main.cpp)
target_link_libraries(acessna_run PRIVATE acessna)

