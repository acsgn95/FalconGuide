#pragma once

#include "falconguide/core/frames.hpp"
#include "falconguide/core/sensors/common.hpp"
#include "falconguide/core/time.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <vector>

namespace falconguide::core {

enum class WheelEncoderLayout {
  Unknown,
  SingleWheel,
  DifferentialDrive,
  Ackermann,
  FourWheelIndependent
};

struct WheelEncoderCalibration {
  WheelEncoderLayout layout{WheelEncoderLayout::Unknown};
  double ticks_per_revolution{0.0};
  double wheel_radius_m{0.0};
  double wheel_base_m{0.0};
  double track_width_m{0.0};
  NoiseDensity tick_noise;
};

struct WheelEncoderMeasurement {
  Timestamp timestamp;
  std::vector<std::int64_t> wheel_ticks;
  std::vector<double> wheel_angular_rates_radps;
  MeasurementValidity validity{MeasurementValidity::Valid};
};

struct WheelOdometryMeasurement {
  Timestamp timestamp;
  Vec3<BodyFrame> linear_velocity_body_mps;
  Vec3<BodyFrame> angular_rate_body_radps;
  Eigen::Matrix<double, 6, 6> covariance{Eigen::Matrix<double, 6, 6>::Zero()};
  MeasurementValidity validity{MeasurementValidity::Valid};
};

}  // namespace falconguide::core
