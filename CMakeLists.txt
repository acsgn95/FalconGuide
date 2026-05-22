cmake_minimum_required(VERSION 3.23)

# =============================================================================
# Version Management
# =============================================================================
# Keep VERSION as the single source of truth for semantic project versioning.
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/VERSION" FALCONGUIDE_VERSION_FILE)
string(STRIP "${FALCONGUIDE_VERSION_FILE}" FALCONGUIDE_VERSION)

string(REGEX MATCH "^([0-9]+)\\.([0-9]+)\\.([0-9]+)([-+].*)?$" _ "${FALCONGUIDE_VERSION}")
if(NOT CMAKE_MATCH_0)
  message(FATAL_ERROR "Invalid FalconGuide VERSION '${FALCONGUIDE_VERSION}'. Expected MAJOR.MINOR.PATCH.")
endif()

set(FALCONGUIDE_VERSION_MAJOR "${CMAKE_MATCH_1}")
set(FALCONGUIDE_VERSION_MINOR "${CMAKE_MATCH_2}")
set(FALCONGUIDE_VERSION_PATCH "${CMAKE_MATCH_3}")

find_package(Git QUIET)
if(GIT_FOUND)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --short=7 HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE FALCONGUIDE_GIT_COMMIT_SHORT
    OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --abbrev-ref HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE FALCONGUIDE_GIT_BRANCH
    OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
else()
  set(FALCONGUIDE_GIT_COMMIT_SHORT "unknown")
  set(FALCONGUIDE_GIT_BRANCH "unknown")
endif()

message(STATUS "FalconGuide version: ${FALCONGUIDE_VERSION}")
message(STATUS "  Git commit: ${FALCONGUIDE_GIT_COMMIT_SHORT}")
message(STATUS "  Git branch: ${FALCONGUIDE_GIT_BRANCH}")

project(
  FalconGuide
  VERSION ${FALCONGUIDE_VERSION}
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
add_subdirectory(src/ui)
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
