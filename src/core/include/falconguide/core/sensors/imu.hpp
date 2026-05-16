#pragma once

#include "falconguide/core/frames.hpp"
#include "falconguide/core/sensors/common.hpp"
#include "falconguide/core/time.hpp"

#include <Eigen/Core>

#include <optional>

namespace falconguide::core {

struct ImuCalibration {
  Vec3<ImuFrame> accelerometer_bias_mps2{};
  Vec3<ImuFrame> gyroscope_bias_radps{};
  Eigen::Matrix3d accelerometer_scale{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d gyroscope_scale{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d accelerometer_misalignment{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d gyroscope_misalignment{Eigen::Matrix3d::Identity()};
  NoiseDensity accelerometer_noise;
  NoiseDensity gyroscope_noise;
  Transform<ImuFrame, BodyFrame> imu_to_body;
};

struct ImuMeasurement {
  Timestamp timestamp;
  Vec3<ImuFrame> specific_force_mps2;
  Vec3<ImuFrame> angular_rate_radps;
  std::optional<double> temperature_celsius;
  MeasurementValidity validity{MeasurementValidity::Valid};
};

}  // namespace falconguide::core
