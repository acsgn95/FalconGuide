add_library(falconguide_estimation STATIC
  src/backends/ekf/ekf_estimator.cpp
  src/backends/ekf/measurement_models/gnss_loosely_coupled.cpp
  src/backends/ekf/measurement_models/gnss_tightly_coupled.cpp
)
add_library(FalconGuide::estimation ALIAS falconguide_estimation)

target_include_directories(
  falconguide_estimation
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
  falconguide_estimation
  PUBLIC
    FalconGuide::core
)
