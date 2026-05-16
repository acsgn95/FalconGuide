#pragma once

#include "falconguide/core/frames.hpp"
#include "falconguide/core/sensors/common.hpp"
#include "falconguide/core/time.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace falconguide::core {

enum class ExternalMeasurementSource {
  Unknown,
  MotionCapture,
  Fiducial,
  Uwb,
  ExternalSlam,
  VehicleCan,
  OperatorInput
};

struct ExternalPoseMeasurement {
  Timestamp timestamp;
  ExternalMeasurementSource source{ExternalMeasurementSource::Unknown};
  Vec3<EcefFrame> position_ecef_m;
  Eigen::Quaterniond orientation_body_to_ecef{Eigen::Quaterniond::Identity()};
  Eigen::Matrix<double, 6, 6> covariance{Eigen::Matrix<double, 6, 6>::Zero()};
  MeasurementValidity validity{MeasurementValidity::Valid};
};

struct ExternalVelocityMeasurement {
  Timestamp timestamp;
  ExternalMeasurementSource source{ExternalMeasurementSource::Unknown};
  Vec3<BodyFrame> linear_velocity_body_mps;
  Vec3<BodyFrame> angular_rate_body_radps;
  Eigen::Matrix<double, 6, 6> covariance{Eigen::Matrix<double, 6, 6>::Zero()};
  MeasurementValidity validity{MeasurementValidity::Valid};
};

struct ExternalOdometryMeasurement {
  Timestamp timestamp;
  ExternalMeasurementSource source{ExternalMeasurementSource::Unknown};
  Vec3<EcefFrame> position_ecef_m;
  Vec3<EcefFrame> velocity_ecef_mps;
  Eigen::Quaterniond orientation_body_to_ecef{Eigen::Quaterniond::Identity()};
  Eigen::Matrix<double, 9, 9> covariance{Eigen::Matrix<double, 9, 9>::Zero()};
  MeasurementValidity validity{MeasurementValidity::Valid};
};

}  // namespace falconguide::core
