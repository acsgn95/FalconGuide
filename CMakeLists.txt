cmake_minimum_required(VERSION 3.23)

project(
  FalconGuide
  VERSION 0.1.0
  DESCRIPTION "Visual-Inertial GNSS Navigation System"
  LANGUAGES CXX)

option(FALCON_GUIDE_BUILD_TESTS "Build FalconGuide tests" ON)
option(FALCON_GUIDE_FETCH_DEPS "Fetch missing dependencies on supported platforms" OFF)
option(FALCON_GUIDE_BUILD_DOCS "Add a Doxygen documentation target" OFF)

# Always build spdlog from source via FetchContent to avoid ABI mismatches with
# pre-built binaries (e.g. conda packages compiled against a different libstdc++).
set(CMAKE_DISABLE_FIND_PACKAGE_spdlog ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(FalconGuideDependencies)

add_subdirectory(src/core)
add_subdirectory(src/logger)
add_subdirectory(src/estimation)
add_subdirectory(src/io)
add_subdirectory(src/app)
add_subdirectory(exe)

if(FALCON_GUIDE_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

if(FALCON_GUIDE_BUILD_DOCS)
  find_package(Doxygen REQUIRED)
  add_custom_target(
    docs
    COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Generating FalconGuide API documentation with Doxygen"
    VERBATIM)
endif()
