#include "falconguide/core/frames.hpp"
#include "falconguide/core/sensor_types.hpp"
#include "falconguide/core/time.hpp"

#include <cassert>
#include <chrono>
#include <type_traits>

using namespace falconguide::core;

int main() {
  static_assert(!std::is_constructible_v<Vec3<EcefFrame>, Vec3<EnuFrame>>);
  static_assert(!std::is_assignable_v<Vec3<EcefFrame>&, Vec3<EnuFrame>>);

  const Vec3<EcefFrame> ecef_a(1.0, 2.0, 3.0);
  const Vec3<EcefFrame> ecef_b(4.0, 5.0, 6.0);
  const Vec3<EcefFrame> ecef_sum = ecef_a + ecef_b;
  assert(ecef_sum.x() == 5.0);
  assert(ecef_sum.y() == 7.0);
  assert(ecef_sum.z() == 9.0);

  const Transform<EcefFrame, EnuFrame> ecef_to_enu(
      Eigen::Quaterniond::Identity(),
      Vec3<EnuFrame>(10.0, 0.0, 0.0));
  const Vec3<EnuFrame> enu_point = ecef_to_enu * ecef_a;
  assert(enu_point.x() == 11.0);

  const MonotonicTime t0(std::chrono::nanoseconds(1'000'000'000));
  const MonotonicTime t1(std::chrono::nanoseconds(1'500'000'000));
  assert((t1 - t0).seconds() == 0.5);

  ImuMeasurement imu;
  imu.specific_force_mps2 = Vec3<ImuFrame>(0.0, 0.0, -9.80665);
  imu.angular_rate_radps = Vec3<ImuFrame>(0.01, 0.02, 0.03);
  assert(imu.validity == MeasurementValidity::Valid);

  MagnetometerCalibration magnetometer_calibration;
  magnetometer_calibration.hard_iron_bias_tesla = Vec3<MagnetometerFrame>(1e-6, 2e-6, 3e-6);

  MagnetometerMeasurement magnetometer;
  magnetometer.magnetic_field_tesla = Vec3<MagnetometerFrame>(25e-6, 0.0, 40e-6);
  assert(magnetometer.validity == MeasurementValidity::Valid);
  assert(magnetometer.magnetic_field_tesla.z() == 40e-6);

  BarometerCalibration barometer_calibration;
  barometer_calibration.pressure_bias_pa = 1.5;

  BarometerMeasurement barometer;
  barometer.pressure_pa = 101'325.0;
  barometer.temperature_celsius = 20.0;
  barometer.altitude_m = 0.0;
  assert(barometer.validity == MeasurementValidity::Valid);
  assert(barometer.pressure_pa == 101'325.0);

  RadarAltimeterCalibration radar_altimeter_calibration;
  radar_altimeter_calibration.min_range_m = 0.3;
  radar_altimeter_calibration.max_range_m = 120.0;
  radar_altimeter_calibration.beam_width_rad = 0.35;
  radar_altimeter_calibration.mode = RadarAltimeterMode::FrequencyModulatedContinuousWave;

  RadarAltimeterMeasurement radar_altimeter;
  radar_altimeter.range_m = 42.0;
  radar_altimeter.signal_to_noise_db = 18.0;
  radar_altimeter.surface = RadarAltimeterSurface::Ground;
  assert(radar_altimeter.validity == MeasurementValidity::Valid);
  assert(radar_altimeter.range_m == 42.0);

  WheelOdometryMeasurement wheel_odometry;
  wheel_odometry.linear_velocity_body_mps = Vec3<BodyFrame>(3.0, 0.0, 0.0);
  assert(wheel_odometry.linear_velocity_body_mps.x() == 3.0);

  AirspeedMeasurement airspeed;
  airspeed.type = AirspeedType::TrueAirspeed;
  airspeed.airspeed_mps = 24.0;
  assert(airspeed.airspeed_mps == 24.0);

  RangeFinderMeasurement range_finder;
  range_finder.type = RangeFinderType::Lidar;
  range_finder.range_m = 12.5;
  assert(range_finder.range_m == 12.5);

  OpticalFlowMeasurement optical_flow;
  optical_flow.integrated_flow_x_rad = 0.01;
  optical_flow.ground_distance_m = 8.0;
  assert(optical_flow.validity == MeasurementValidity::Valid);

  DvlMeasurement dvl;
  dvl.velocity_mps = Vec3<DvlFrame>(0.4, 0.0, 0.0);
  dvl.reference = DvlReference::BottomTrack;
  assert(dvl.velocity_mps.x() == 0.4);

  EchoSounderMeasurement echo_sounder;
  echo_sounder.depth_or_range_m = 30.0;
  assert(echo_sounder.depth_or_range_m == 30.0);

  ExternalOdometryMeasurement external_odometry;
  external_odometry.source = ExternalMeasurementSource::ExternalSlam;
  external_odometry.position_ecef_m = Vec3<EcefFrame>(1.0, 2.0, 3.0);
  assert(external_odometry.source == ExternalMeasurementSource::ExternalSlam);

  GnssObservationEpoch epoch;
  epoch.observations.push_back(
      GnssRawObservation{SatelliteId{GnssConstellation::Gps, 3}, GnssSignal::L1, 22'000'000.0});
  assert(epoch.observations.size() == 1);

  CameraCalibration odometry_camera;
  odometry_camera.camera_name = "front_odometry";
  odometry_camera.role = CameraRole::VisualOdometry;
  odometry_camera.intrinsics.width = 1280;
  odometry_camera.intrinsics.height = 720;
  assert(odometry_camera.role == CameraRole::VisualOdometry);

  CameraCalibration geo_camera;
  geo_camera.camera_name = "down_geo_reference";
  geo_camera.role = CameraRole::GeoReference;
  assert(geo_camera.role == CameraRole::GeoReference);

  GeoReferenceImageMeasurement geo_image;
  geo_image.camera_name = geo_camera.camera_name;
  geo_image.frame_id = 42;
  geo_image.approximate_position_ecef_m = Vec3<EcefFrame>(1.0, 2.0, 3.0);
  geo_image.ground_sample_distance_m = 0.25;
  assert(geo_image.validity == MeasurementValidity::Valid);

  return 0;
}
