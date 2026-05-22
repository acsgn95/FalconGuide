#pragma once

/**
 * @file range.hpp
 * @brief Altimeter, range-finder, optical-flow, DVL, and echo-sounder types.
 */

#include "falconguide/core/frames.hpp"
#include "falconguide/core/sensors/common.hpp"
#include "falconguide/core/time.hpp"

#include <Eigen/Core>

#include <optional>

namespace falconguide::core {

/// @brief Radar altimeter measurement technology.
enum class RadarAltimeterMode {
  Unknown,                          ///< Mode is not known.
  FrequencyModulatedContinuousWave, ///< FMCW radar altimeter.
  Pulsed,                           ///< Pulsed radar altimeter.
  LidarLikeTimeOfFlight             ///< Time-of-flight lidar-like ranging.
};

/// @brief Surface classification reported by an altimeter.
enum class RadarAltimeterSurface {
  Unknown,    ///< Surface type is not known.
  Ground,     ///< Ground return.
  Water,      ///< Water return.
  Vegetation, ///< Vegetation return.
  Building,   ///< Building or structure return.
  Invalid     ///< Return is invalid.
};

/// @brief Calibration and operating limits for a radar altimeter.
struct RadarAltimeterCalibration {
  Transform<RadarAltimeterFrame, BodyFrame>
      radar_altimeter_to_body; ///< Sensor to body transform.
  double range_bias_m{0.0};    ///< Additive range bias.
  double range_scale{1.0};     ///< Multiplicative range scale.
  double min_range_m{0.0};     ///< Minimum valid range.
  double max_range_m{0.0};     ///< Maximum valid range.
  double beam_width_rad{0.0};  ///< Beam width in radians.
  NoiseDensity range_noise;    ///< Range noise model.
  RadarAltimeterMode mode{
      RadarAltimeterMode::Unknown}; ///< Altimeter operating mode.
};

/// @brief Timestamped radar-altimeter range measurement.
struct RadarAltimeterMeasurement {
  Timestamp timestamp; ///< Sample timestamp.
  double range_m{0.0}; ///< Slant or vertical range in meters.
  std::optional<double>
      vertical_velocity_mps; ///< Optional vertical velocity estimate.
  std::optional<double> signal_to_noise_db; ///< Optional signal-to-noise ratio.
  std::optional<double> return_strength; ///< Optional return-strength metric.
  RadarAltimeterSurface surface{
      RadarAltimeterSurface::Unknown}; ///< Classified reflecting surface.
  MeasurementValidity validity{
      MeasurementValidity::Valid}; ///< Source validity state.
};

/// @brief Generic range-finder technology.
enum class RangeFinderType {
  Unknown,    ///< Type is not known.
  Laser,      ///< Single-beam laser range finder.
  Lidar,      ///< Lidar range measurement.
  Ultrasonic, ///< Ultrasonic range measurement.
  Sonar,      ///< Sonar range measurement.
  Radar       ///< Radar range measurement.
};

/// @brief Calibration for a generic range finder.
struct RangeFinderCalibration {
  Transform<RangeFinderFrame, BodyFrame>
      range_finder_to_body;                       ///< Sensor to body transform.
  RangeFinderType type{RangeFinderType::Unknown}; ///< Range-finder technology.
  double range_bias_m{0.0};                       ///< Additive range bias.
  double range_scale{1.0};    ///< Multiplicative range scale.
  double min_range_m{0.0};    ///< Minimum valid range.
  double max_range_m{0.0};    ///< Maximum valid range.
  double beam_width_rad{0.0}; ///< Beam width in radians.
  NoiseDensity range_noise;   ///< Range noise model.
};

/// @brief Timestamped generic range-finder measurement.
struct RangeFinderMeasurement {
  Timestamp timestamp; ///< Sample timestamp.
  double range_m{0.0}; ///< Range in meters.
  std::optional<double>
      signal_quality; ///< Optional normalized or device-specific quality.
  RangeFinderType type{RangeFinderType::Unknown}; ///< Range-finder technology.
  MeasurementValidity validity{
      MeasurementValidity::Valid}; ///< Source validity state.
};

/// @brief Calibration for an optical-flow sensor.
struct OpticalFlowCalibration {
  Transform<OpticalFlowFrame, BodyFrame>
      optical_flow_to_body;      ///< Sensor to body transform.
  double focal_length_x_px{0.0}; ///< Horizontal focal length in pixels.
  double focal_length_y_px{0.0}; ///< Vertical focal length in pixels.
  NoiseDensity flow_noise;       ///< Optical-flow noise model.
};

/// @brief Timestamped integrated optical-flow measurement.
struct OpticalFlowMeasurement {
  Timestamp timestamp; ///< Sample timestamp.
  double integrated_flow_x_rad{
      0.0}; ///< Integrated image flow around x in radians.
  double integrated_flow_y_rad{
      0.0}; ///< Integrated image flow around y in radians.
  std::optional<double> integration_time_s; ///< Optional integration interval.
  std::optional<double>
      ground_distance_m; ///< Optional distance to ground used for scaling.
  std::optional<Vec3<BodyFrame>>
      angular_rate_body_radps;   ///< Optional body angular-rate compensation.
  std::optional<double> quality; ///< Optional normalized quality score.
  MeasurementValidity validity{
      MeasurementValidity::Valid}; ///< Source validity state.
};

/// @brief Doppler Velocity Log tracking reference.
enum class DvlReference {
  Unknown,     ///< Reference is not known.
  BottomTrack, ///< Velocity measured relative to seabed.
  WaterTrack   ///< Velocity measured relative to water mass.
};

/// @brief Calibration for a Doppler Velocity Log.
struct DvlCalibration {
  Transform<DvlFrame, BodyFrame> dvl_to_body; ///< DVL to body transform.
  NoiseDensity velocity_noise;                ///< Velocity noise model.
};

/// @brief Timestamped DVL velocity measurement.
struct DvlMeasurement {
  Timestamp timestamp;         ///< Sample timestamp.
  Vec3<DvlFrame> velocity_mps; ///< Velocity in the DVL frame.
  Eigen::Matrix3d velocity_covariance_m2ps2{
      Eigen::Matrix3d::Zero()};                  ///< Velocity covariance.
  DvlReference reference{DvlReference::Unknown}; ///< Tracking reference.
  std::optional<double> altitude_m; ///< Optional altitude above seabed.
  std::optional<double> quality;    ///< Optional normalized quality score.
  MeasurementValidity validity{
      MeasurementValidity::Valid}; ///< Source validity state.
};

/// @brief Calibration for an echo sounder.
struct EchoSounderCalibration {
  Transform<EchoSounderFrame, BodyFrame>
      echo_sounder_to_body; ///< Echo-sounder to body transform.
  double range_bias_m{0.0}; ///< Additive range bias.
  double range_scale{1.0};  ///< Multiplicative range scale.
  NoiseDensity range_noise; ///< Range noise model.
};

/// @brief Timestamped echo-sounder depth or range measurement.
struct EchoSounderMeasurement {
  Timestamp timestamp;                   ///< Sample timestamp.
  double depth_or_range_m{0.0};          ///< Depth or range in meters.
  std::optional<double> sound_speed_mps; ///< Optional sound-speed correction.
  std::optional<double> return_strength; ///< Optional return-strength metric.
  MeasurementValidity validity{
      MeasurementValidity::Valid}; ///< Source validity state.
};

} // namespace falconguide::core
