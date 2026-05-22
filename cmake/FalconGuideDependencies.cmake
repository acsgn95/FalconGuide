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

# ── nlohmann/json ─────────────────────────────────────────────────────────────
find_package(nlohmann_json CONFIG QUIET)

if(NOT nlohmann_json_FOUND)
  FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.3
    GIT_SHALLOW    TRUE
  )
  set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
  set(JSON_Install    OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(nlohmann_json)
endif()

if(NOT TARGET nlohmann_json::nlohmann_json)
  message(FATAL_ERROR "nlohmann_json not found and could not be fetched.")
endif()

# ── Ceres Solver (optional — sliding-window backend) ──────────────────────────
option(FALCONGUIDE_ENABLE_CERES "Enable Ceres sliding-window estimator backend" ON)

if(FALCONGUIDE_ENABLE_CERES)
  find_package(Ceres QUIET)

  if(NOT Ceres_FOUND)
    message(STATUS "Ceres not found via find_package — fetching 2.2.0")
    FetchContent_Declare(
      ceres_solver
      GIT_REPOSITORY https://ceres-solver.googlesource.com/ceres-solver.git
      GIT_TAG        2.2.0
      GIT_SHALLOW    TRUE
    )
    set(BUILD_TESTING      OFF CACHE BOOL "" FORCE)
    set(BUILD_EXAMPLES     OFF CACHE BOOL "" FORCE)
    set(BUILD_BENCHMARKS   OFF CACHE BOOL "" FORCE)
    set(PROVIDE_UNINSTALL_TARGET OFF CACHE BOOL "" FORCE)
    set(MINIGLOG           ON  CACHE BOOL "" FORCE)   # avoid glog dependency
    FetchContent_MakeAvailable(ceres_solver)
  else()
    message(STATUS "Ceres found: ${CERES_VERSION}")
  endif()
endif()

# ── GTSAM (optional — factor-graph backend) ───────────────────────────────────
option(FALCONGUIDE_ENABLE_GTSAM "Enable GTSAM factor-graph estimator backend" ON)

if(FALCONGUIDE_ENABLE_GTSAM)
  find_package(GTSAM QUIET)

  if(NOT GTSAM_FOUND)
    message(STATUS "GTSAM not found via find_package — fetching 4.2.0")
    FetchContent_Declare(
      gtsam
      GIT_REPOSITORY https://github.com/borglab/gtsam.git
      GIT_TAG        4.2.0
      GIT_SHALLOW    TRUE
    )
    set(GTSAM_BUILD_TESTS            OFF CACHE BOOL "" FORCE)
    set(GTSAM_BUILD_EXAMPLES         OFF CACHE BOOL "" FORCE)
    set(GTSAM_BUILD_DOCS             OFF CACHE BOOL "" FORCE)
    set(GTSAM_USE_SYSTEM_EIGEN       ON  CACHE BOOL "" FORCE)
    set(GTSAM_BUILD_WITH_MARCH_NATIVE OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(gtsam)
  else()
    message(STATUS "GTSAM found: ${GTSAM_VERSION}")
  endif()
endif()
