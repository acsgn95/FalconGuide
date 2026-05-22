#pragma once

/**
 * @file environment.hpp
 * @brief Environmental and air-data sensor measurements.
 */

#include "falconguide/core/frames.hpp"
#include "falconguide/core/sensors/common.hpp"
#include "falconguide/core/time.hpp"

#include <Eigen/Core>

#include <optional>

namespace falconguide::core {

/// @brief Calibration for a 3-axis magnetometer.
struct MagnetometerCalibration {
  Vec3<MagnetometerFrame> hard_iron_bias_tesla{}; ///< Additive hard-iron bias.
  Eigen::Matrix3d soft_iron_scale{
      Eigen::Matrix3d::Identity()}; ///< Soft-iron scale matrix.
  Eigen::Matrix3d misalignment{
      Eigen::Matrix3d::Identity()};  ///< Sensor-axis misalignment matrix.
  NoiseDensity magnetic_field_noise; ///< Magnetic-field noise model.
  Transform<MagnetometerFrame, BodyFrame>
      magnetometer_to_body; ///< Magnetometer to body transform.
};

/// @brief Timestamped magnetometer field measurement.
struct MagnetometerMeasurement {
  Timestamp timestamp;                          ///< Sample timestamp.
  Vec3<MagnetometerFrame> magnetic_field_tesla; ///< Magnetic field in Tesla.
  std::optional<double> temperature_celsius; ///< Optional sensor temperature.
  MeasurementValidity validity{
      MeasurementValidity::Valid}; ///< Source validity state.
};

/// @brief Calibration for a static pressure barometer.
struct BarometerCalibration {
  double pressure_bias_pa{0.0}; ///< Additive pressure bias in Pascal.
  double pressure_scale{1.0};   ///< Multiplicative pressure scale factor.
  NoiseDensity pressure_noise;  ///< Pressure noise model.
};

/// @brief Timestamped barometer pressure measurement.
struct BarometerMeasurement {
  Timestamp timestamp;     ///< Sample timestamp.
  double pressure_pa{0.0}; ///< Static pressure in Pascal.
  std::optional<double>
      temperature_celsius;          ///< Optional ambient or sensor temperature.
  std::optional<double> altitude_m; ///< Optional precomputed pressure altitude.
  MeasurementValidity validity{
      MeasurementValidity::Valid}; ///< Source validity state.
};

/// @brief Airspeed value interpretation.
enum class AirspeedType {
  Unknown,     ///< Airspeed type is not known.
  Indicated,   ///< Indicated airspeed.
  Calibrated,  ///< Calibrated airspeed.
  TrueAirspeed ///< True airspeed.
};

/// @brief Calibration for an airspeed sensor.
struct AirspeedCalibration {
  double differential_pressure_bias_pa{
      0.0}; ///< Additive differential-pressure bias.
  double differential_pressure_scale{
      1.0};                      ///< Differential-pressure scale factor.
  double airspeed_bias_mps{0.0}; ///< Additive airspeed bias in m/s.
  double airspeed_scale{1.0};    ///< Airspeed scale factor.
  NoiseDensity airspeed_noise;   ///< Airspeed noise model.
};

/// @brief Timestamped airspeed measurement.
struct AirspeedMeasurement {
  Timestamp timestamp; ///< Sample timestamp.
  AirspeedType type{
      AirspeedType::Unknown}; ///< Type of airspeed value supplied.
  double airspeed_mps{0.0};   ///< Airspeed in meters per second.
  std::optional<double>
      differential_pressure_pa;             ///< Optional differential pressure.
  std::optional<double> static_pressure_pa; ///< Optional static pressure.
  std::optional<double> air_temperature_celsius; ///< Optional air temperature.
  MeasurementValidity validity{
      MeasurementValidity::Valid}; ///< Source validity state.
};

} // namespace falconguide::core
