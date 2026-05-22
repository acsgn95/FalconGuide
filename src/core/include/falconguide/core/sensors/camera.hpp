#pragma once

#include "falconguide/core/frames.hpp"
#include "falconguide/core/sensors/common.hpp"
#include "falconguide/core/time.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <optional>
#include <string>

namespace falconguide::core {

struct CameraIntrinsics {
  std::uint32_t width{0};
  std::uint32_t height{0};
  Eigen::VectorXd parameters;
  std::string model;
};

enum class CameraRole {
  Unknown,
  VisualOdometry,
  GeoReference,
  Mapping,
  Landing,
  Inspection
};

struct CameraCalibration {
  std::string camera_name;
  CameraRole role{CameraRole::Unknown};
  CameraIntrinsics intrinsics;
  Transform<CameraFrame, BodyFrame> camera_to_body;
};

struct CameraConfig {
  std::string camera_name;
  CameraRole role{CameraRole::Unknown};
  double nominal_frame_rate_hz{0.0};
  bool hardware_synchronized{false};
};

struct CameraFrameMeasurement {
  Timestamp timestamp;
  std::uint64_t frame_id{0};
  std::string camera_name;
  CameraRole role{CameraRole::Unknown};
  std::optional<double> exposure_time_s;
  std::optional<double> gain;
  MeasurementValidity validity{MeasurementValidity::Valid};
};

struct GeoReferenceImageMeasurement {
  Timestamp timestamp;
  std::uint64_t frame_id{0};
  std::string camera_name;
  std::optional<Vec3<EcefFrame>> approximate_position_ecef_m;
  std::optional<double> ground_sample_distance_m;
  std::optional<double> yaw_prior_rad;
  MeasurementValidity validity{MeasurementValidity::Valid};
};

}  // namespace falconguide::core
