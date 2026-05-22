add_library(falconguide_estimation STATIC
  # Pipeline / system
  src/estimator_pipeline.cpp
  src/navigation_system.cpp
  # EKF backend
  src/backends/ekf/ekf_estimator.cpp
  src/backends/ekf/measurement_models/gnss_loosely_coupled.cpp
  src/backends/ekf/measurement_models/gnss_tightly_coupled.cpp
  src/backends/ekf/measurement_models/barometer.cpp
  src/backends/ekf/measurement_models/magnetometer.cpp
  src/backends/ekf/measurement_models/airspeed.cpp
  src/backends/ekf/measurement_models/wheel_odometry.cpp
  src/backends/ekf/measurement_models/radar_altimeter.cpp
  src/backends/ekf/measurement_models/range_finder.cpp
  src/backends/ekf/measurement_models/dvl.cpp
  src/backends/ekf/measurement_models/echo_sounder.cpp
  src/backends/ekf/measurement_models/optical_flow.cpp
  src/backends/ekf/measurement_models/star_tracker.cpp
  src/backends/ekf/measurement_models/aiding_solution.cpp
  src/backends/ekf/measurement_models/external_pose.cpp
  src/backends/ekf/measurement_models/external_velocity.cpp
  src/backends/ekf/measurement_models/external_odometry.cpp
  # UKF backend
  src/backends/ukf/ukf_estimator.cpp
  src/backends/ukf/measurement_models/ukf_gnss_loosely_coupled.cpp
  src/backends/ukf/measurement_models/ukf_barometer.cpp
  src/backends/ukf/measurement_models/ukf_magnetometer.cpp
  src/backends/ukf/measurement_models/ukf_airspeed.cpp
  src/backends/ukf/measurement_models/ukf_wheel_odometry.cpp
  src/backends/ukf/measurement_models/ukf_radar_altimeter.cpp
  src/backends/ukf/measurement_models/ukf_dvl.cpp
  src/backends/ukf/measurement_models/ukf_echo_sounder.cpp
  src/backends/ukf/measurement_models/ukf_optical_flow.cpp
  src/backends/ukf/measurement_models/ukf_star_tracker.cpp
  src/backends/ukf/measurement_models/ukf_aiding_solution.cpp
  src/backends/ukf/measurement_models/ukf_external_pose.cpp
  src/backends/ukf/measurement_models/ukf_external_velocity.cpp
  src/backends/ukf/measurement_models/ukf_external_odometry.cpp
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
    FalconGuide::logger
)
