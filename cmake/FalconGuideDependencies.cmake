include(FetchContent)

find_package(Eigen3 CONFIG QUIET)
if(NOT Eigen3_FOUND)
  find_package(Eigen3 QUIET)
endif()

if(NOT Eigen3_FOUND AND FALCON_GUIDE_FETCH_DEPS)
  FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(eigen)
endif()

if(NOT TARGET Eigen3::Eigen)
  message(FATAL_ERROR "Eigen3 was not found. Install it with vcpkg/system packages, or configure with -DFALCON_GUIDE_FETCH_DEPS=ON.")
endif()
