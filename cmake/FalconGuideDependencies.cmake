include(FetchContent)

# ── Eigen3 ────────────────────────────────────────────────────────────────────
find_package(Eigen3 CONFIG QUIET)
if(NOT Eigen3_FOUND)
  find_package(Eigen3 QUIET)
endif()

if(NOT Eigen3_FOUND)
  FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
    GIT_SHALLOW    TRUE
  )
  FetchContent_MakeAvailable(eigen)
endif()

if(NOT TARGET Eigen3::Eigen)
  message(FATAL_ERROR "Eigen3 not found and could not be fetched.")
endif()

# ── spdlog ────────────────────────────────────────────────────────────────────
find_package(spdlog CONFIG QUIET)

if(NOT spdlog_FOUND)
  FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG        v1.14.1
    GIT_SHALLOW    TRUE
  )
  set(SPDLOG_BUILD_EXAMPLE OFF CACHE BOOL "" FORCE)
  set(SPDLOG_BUILD_TESTS   OFF CACHE BOOL "" FORCE)
  set(SPDLOG_INSTALL       OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(spdlog)
endif()

if(NOT TARGET spdlog::spdlog)
  message(FATAL_ERROR "spdlog not found and could not be fetched.")
endif()
