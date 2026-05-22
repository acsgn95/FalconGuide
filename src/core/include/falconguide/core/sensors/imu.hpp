#pragma once

/**
 * @file imu.hpp
 * @brief IMU calibration and measurement types.
 */

#include "falconguide/core/frames.hpp"
#include "falconguide/core/sensors/common.hpp"
#include "falconguide/core/time.hpp"

#include <Eigen/Core>

#include <optional>

namespace falconguide::core {

/// @brief IMU calibration parameters needed to correct raw accelerometer and
/// gyroscope readings.
struct ImuCalibration {
  Vec3<ImuFrame>
      accelerometer_bias_mps2{}; ///< Accelerometer additive bias in m/s^2.
  Vec3<ImuFrame> gyroscope_bias_radps{}; ///< Gyroscope additive bias in rad/s.
  Eigen::Matrix3d accelerometer_scale{
      Eigen::Matrix3d::Identity()}; ///< Accelerometer scale correction.
  Eigen::Matrix3d gyroscope_scale{
      Eigen::Matrix3d::Identity()}; ///< Gyroscope scale correction.
  Eigen::Matrix3d accelerometer_misalignment{
      Eigen::Matrix3d::Identity()}; ///< Accelerometer axis misalignment.
  Eigen::Matrix3d gyroscope_misalignment{
      Eigen::Matrix3d::Identity()}; ///< Gyroscope axis misalignment.
  NoiseDensity accelerometer_noise; ///< Accelerometer noise model.
  NoiseDensity gyroscope_noise;     ///< Gyroscope noise model.
  Transform<ImuFrame, BodyFrame>
      imu_to_body; ///< Rigid transform from IMU frame to body frame.
};

/// @brief One timestamped IMU sample.
struct ImuMeasurement {
  Timestamp timestamp;                ///< Sample timestamp.
  Vec3<ImuFrame> specific_force_mps2; ///< Specific force measured by
                                      ///< accelerometers in m/s^2.
  Vec3<ImuFrame>
      angular_rate_radps; ///< Angular rate measured by gyroscopes in rad/s.
  std::optional<double> temperature_celsius; ///< Optional sensor temperature.
  MeasurementValidity validity{
      MeasurementValidity::Valid}; ///< Source validity state.
};

} // namespace falconguide::core
