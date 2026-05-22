#pragma once

/**
 * @file odometry.hpp
 * @brief Wheel encoder and wheel odometry measurement types.
 */

#include "falconguide/core/frames.hpp"
#include "falconguide/core/sensors/common.hpp"
#include "falconguide/core/time.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <vector>

namespace falconguide::core {

/// @brief Kinematic layout of wheel encoders.
enum class WheelEncoderLayout {
  Unknown,             ///< Layout is not known.
  SingleWheel,         ///< Single wheel encoder.
  DifferentialDrive,   ///< Differential-drive pair.
  Ackermann,           ///< Ackermann steering layout.
  FourWheelIndependent ///< Four independent wheel encoders.
};

/// @brief Calibration for wheel encoder odometry.
struct WheelEncoderCalibration {
  WheelEncoderLayout layout{WheelEncoderLayout::Unknown}; ///< Encoder layout.
  double ticks_per_revolution{0.0}; ///< Encoder ticks per wheel revolution.
  double wheel_radius_m{0.0};       ///< Wheel radius in meters.
  double wheel_base_m{0.0};         ///< Wheel base in meters.
  double track_width_m{0.0};        ///< Track width in meters.
  NoiseDensity tick_noise;          ///< Tick-count noise model.
};

/// @brief Timestamped raw wheel encoder measurement.
struct WheelEncoderMeasurement {
  Timestamp timestamp;                   ///< Sample timestamp.
  std::vector<std::int64_t> wheel_ticks; ///< Per-wheel tick counts.
  std::vector<double>
      wheel_angular_rates_radps; ///< Optional per-wheel angular rates.
  MeasurementValidity validity{
      MeasurementValidity::Valid}; ///< Source validity state.
};

/// @brief Timestamped body-frame velocity estimate from wheel odometry.
struct WheelOdometryMeasurement {
  Timestamp timestamp;                      ///< Sample timestamp.
  Vec3<BodyFrame> linear_velocity_body_mps; ///< Body-frame linear velocity.
  Vec3<BodyFrame> angular_rate_body_radps;  ///< Body-frame angular rate.
  Eigen::Matrix<double, 6, 6> covariance{
      Eigen::Matrix<double, 6, 6>::Zero()}; ///< Velocity covariance.
  MeasurementValidity validity{
      MeasurementValidity::Valid}; ///< Source validity state.
};

} // namespace falconguide::core
