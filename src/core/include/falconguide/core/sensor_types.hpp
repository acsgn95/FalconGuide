#pragma once

#include "falconguide/core/frames.hpp"
#include "falconguide/core/time.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace falconguide::core {

enum class MeasurementValidity {
  Valid,
  Saturated,
  Dropped,
  OutOfOrder,
  Unknown
};

struct NoiseDensity {
  double continuous_noise_density{0.0};
  double random_walk{0.0};
};

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

enum class GnssConstellation {
  Gps,
  Glonass,
  Galileo,
  BeiDou,
  Qzss,
  Sbas,
  Unknown
};

enum class GnssSignal {
  L1,
  L2,
  L5,
  E1,
  E5a,
  E5b,
  B1,
  B2,
  Unknown
};

enum class GnssFixType {
  NoFix,
  Single,
  Differential,
  RtkFloat,
  RtkFixed,
  PrecisePointPositioning
};

struct SatelliteId {
  GnssConstellation constellation{GnssConstellation::Unknown};
  std::uint16_t prn{0};
};

struct GnssRawObservation {
  SatelliteId satellite;
  GnssSignal signal{GnssSignal::Unknown};
  double pseudorange_m{0.0};
  std::optional<double> carrier_phase_cycles;
  std::optional<double> doppler_hz;
  std::optional<double> carrier_to_noise_density_dbhz;
  std::optional<double> lock_time_s;
  MeasurementValidity validity{MeasurementValidity::Valid};
};

struct GnssObservationEpoch {
  Timestamp timestamp;
  std::vector<GnssRawObservation> observations;
};

struct GnssSolution {
  Timestamp timestamp;
  Vec3<EcefFrame> position_ecef_m;
  Vec3<EcefFrame> velocity_ecef_mps;
  Eigen::Matrix3d position_covariance_ecef_m2{Eigen::Matrix3d::Zero()};
  Eigen::Matrix3d velocity_covariance_ecef_m2ps2{Eigen::Matrix3d::Zero()};
  GnssFixType fix_type{GnssFixType::NoFix};
  std::optional<double> horizontal_dop;
  std::optional<double> vertical_dop;
};

struct GnssAntennaCalibration {
  Transform<GnssAntennaFrame, BodyFrame> antenna_to_body;
  Vec3<GnssAntennaFrame> phase_center_offset_m;
};

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
