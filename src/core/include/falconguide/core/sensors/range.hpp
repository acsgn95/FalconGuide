#pragma once

#include "falconguide/core/frames.hpp"
#include "falconguide/core/sensors/common.hpp"
#include "falconguide/core/time.hpp"

#include <Eigen/Core>

#include <optional>

namespace falconguide::core {

enum class RadarAltimeterMode {
  Unknown,
  FrequencyModulatedContinuousWave,
  Pulsed,
  LidarLikeTimeOfFlight
};

enum class RadarAltimeterSurface {
  Unknown,
  Ground,
  Water,
  Vegetation,
  Building,
  Invalid
};

struct RadarAltimeterCalibration {
  Transform<RadarAltimeterFrame, BodyFrame> radar_altimeter_to_body;
  double range_bias_m{0.0};
  double range_scale{1.0};
  double min_range_m{0.0};
  double max_range_m{0.0};
  double beam_width_rad{0.0};
  NoiseDensity range_noise;
  RadarAltimeterMode mode{RadarAltimeterMode::Unknown};
};

struct RadarAltimeterMeasurement {
  Timestamp timestamp;
  double range_m{0.0};
  std::optional<double> vertical_velocity_mps;
  std::optional<double> signal_to_noise_db;
  std::optional<double> return_strength;
  RadarAltimeterSurface surface{RadarAltimeterSurface::Unknown};
  MeasurementValidity validity{MeasurementValidity::Valid};
};

enum class RangeFinderType {
  Unknown,
  Laser,
  Lidar,
  Ultrasonic,
  Sonar,
  Radar
};

struct RangeFinderCalibration {
  Transform<RangeFinderFrame, BodyFrame> range_finder_to_body;
  RangeFinderType type{RangeFinderType::Unknown};
  double range_bias_m{0.0};
  double range_scale{1.0};
  double min_range_m{0.0};
  double max_range_m{0.0};
  double beam_width_rad{0.0};
  NoiseDensity range_noise;
};

struct RangeFinderMeasurement {
  Timestamp timestamp;
  double range_m{0.0};
  std::optional<double> signal_quality;
  RangeFinderType type{RangeFinderType::Unknown};
  MeasurementValidity validity{MeasurementValidity::Valid};
};

struct OpticalFlowCalibration {
  Transform<OpticalFlowFrame, BodyFrame> optical_flow_to_body;
  double focal_length_x_px{0.0};
  double focal_length_y_px{0.0};
  NoiseDensity flow_noise;
};

struct OpticalFlowMeasurement {
  Timestamp timestamp;
  double integrated_flow_x_rad{0.0};
  double integrated_flow_y_rad{0.0};
  std::optional<double> integration_time_s;
  std::optional<double> ground_distance_m;
  std::optional<Vec3<BodyFrame>> angular_rate_body_radps;
  std::optional<double> quality;
  MeasurementValidity validity{MeasurementValidity::Valid};
};

enum class DvlReference {
  Unknown,
  BottomTrack,
  WaterTrack
};

struct DvlCalibration {
  Transform<DvlFrame, BodyFrame> dvl_to_body;
  NoiseDensity velocity_noise;
};

struct DvlMeasurement {
  Timestamp timestamp;
  Vec3<DvlFrame> velocity_mps;
  Eigen::Matrix3d velocity_covariance_m2ps2{Eigen::Matrix3d::Zero()};
  DvlReference reference{DvlReference::Unknown};
  std::optional<double> altitude_m;
  std::optional<double> quality;
  MeasurementValidity validity{MeasurementValidity::Valid};
};

struct EchoSounderCalibration {
  Transform<EchoSounderFrame, BodyFrame> echo_sounder_to_body;
  double range_bias_m{0.0};
  double range_scale{1.0};
  NoiseDensity range_noise;
};

struct EchoSounderMeasurement {
  Timestamp timestamp;
  double depth_or_range_m{0.0};
  std::optional<double> sound_speed_mps;
  std::optional<double> return_strength;
  MeasurementValidity validity{MeasurementValidity::Valid};
};

}  // namespace falconguide::core
