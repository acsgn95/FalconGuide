#pragma once

#include "falconguide/core/frames.hpp"
#include "falconguide/core/sensors/common.hpp"
#include "falconguide/core/time.hpp"

#include <Eigen/Core>

#include <optional>

namespace falconguide::core {

struct MagnetometerCalibration {
  Vec3<MagnetometerFrame> hard_iron_bias_tesla{};
  Eigen::Matrix3d soft_iron_scale{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d misalignment{Eigen::Matrix3d::Identity()};
  NoiseDensity magnetic_field_noise;
  Transform<MagnetometerFrame, BodyFrame> magnetometer_to_body;
};

struct MagnetometerMeasurement {
  Timestamp timestamp;
  Vec3<MagnetometerFrame> magnetic_field_tesla;
  std::optional<double> temperature_celsius;
  MeasurementValidity validity{MeasurementValidity::Valid};
};

struct BarometerCalibration {
  double pressure_bias_pa{0.0};
  double pressure_scale{1.0};
  NoiseDensity pressure_noise;
};

struct BarometerMeasurement {
  Timestamp timestamp;
  double pressure_pa{0.0};
  std::optional<double> temperature_celsius;
  std::optional<double> altitude_m;
  MeasurementValidity validity{MeasurementValidity::Valid};
};

enum class AirspeedType {
  Unknown,
  Indicated,
  Calibrated,
  TrueAirspeed
};

struct AirspeedCalibration {
  double differential_pressure_bias_pa{0.0};
  double differential_pressure_scale{1.0};
  double airspeed_bias_mps{0.0};
  double airspeed_scale{1.0};
  NoiseDensity airspeed_noise;
};

struct AirspeedMeasurement {
  Timestamp timestamp;
  AirspeedType type{AirspeedType::Unknown};
  double airspeed_mps{0.0};
  std::optional<double> differential_pressure_pa;
  std::optional<double> static_pressure_pa;
  std::optional<double> air_temperature_celsius;
  MeasurementValidity validity{MeasurementValidity::Valid};
};

}  // namespace falconguide::core
