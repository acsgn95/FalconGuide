#include "falconguide/core/buffers/time_ordered_buffer.hpp"
#include "falconguide/core/coordinates.hpp"
#include "falconguide/core/diagnostics.hpp"
#include "falconguide/core/frames.hpp"
#include "falconguide/core/math.hpp"
#include "falconguide/core/measurement_helpers.hpp"
#include "falconguide/core/navigation_state.hpp"
#include "falconguide/core/sensor_types.hpp"
#include "falconguide/core/time.hpp"
#include "falconguide/core/time_helpers.hpp"
#include "falconguide/estimation/estimator_interface.hpp"
#include "falconguide/estimation/measurement_variant_helpers.hpp"
#include "falconguide/io/measurement_reader.hpp"
#include "falconguide/io/measurement_writer.hpp"
#include "falconguide/io/nmea/nmea_gga.hpp"
#include "falconguide/io/nmea/nmea_rmc.hpp"
#include "falconguide/io/nmea/nmea_vtg.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <type_traits>
#include <variant>

using namespace falconguide::core;
using namespace falconguide::estimation;

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
  Timestamp timestamp0;
  timestamp0.has_steady = true;
  timestamp0.steady = t0;
  Timestamp timestamp1;
  timestamp1.has_steady = true;
  timestamp1.steady = t1;
  assert(IsBefore(timestamp0, timestamp1));
  assert(IsAfter(timestamp1, timestamp0));
  assert(TimeDifference(timestamp1, timestamp0)->seconds() == 0.5);

  const Lla equator_origin{0.0, 0.0, 0.0};
  const Vec3<EcefFrame> equator_ecef = LlaToEcef(equator_origin);
  assert(Near(equator_ecef.x(), wgs84::kSemiMajorAxisM, 1e-6));
  assert(Near(equator_ecef.y(), 0.0, 1e-6));
  assert(Near(equator_ecef.z(), 0.0, 1e-6));

  const Lla round_trip_lla = EcefToLla(equator_ecef);
  assert(Near(round_trip_lla.latitude_rad, equator_origin.latitude_rad, 1e-12));
  assert(Near(round_trip_lla.longitude_rad, equator_origin.longitude_rad, 1e-12));
  assert(Near(round_trip_lla.altitude_m, equator_origin.altitude_m, 1e-6));

  const LocalTangentPlane tangent_plane(equator_origin);
  const Vec3<EnuFrame> local_enu_point(10.0, 20.0, 30.0);
  const Vec3<EcefFrame> ecef_from_enu = tangent_plane.EnuToEcef(local_enu_point);
  const Vec3<EnuFrame> enu_round_trip = tangent_plane.EcefToEnu(ecef_from_enu);
  assert(Near(enu_round_trip.x(), local_enu_point.x(), 1e-9));
  assert(Near(enu_round_trip.y(), local_enu_point.y(), 1e-9));
  assert(Near(enu_round_trip.z(), local_enu_point.z(), 1e-9));

  assert(Near(DegToRad(180.0), kPi, 1e-12));
  assert(Near(RadToDeg(kPi / 2.0), 90.0, 1e-12));
  assert(Clamp(5, 0, 3) == 3);
  assert(Near(WrapAngleRad(3.0 * kPi), kPi, 1e-12) || Near(WrapAngleRad(3.0 * kPi), -kPi, 1e-12));
  const Eigen::Vector3d vector(1.0, 2.0, 3.0);
  assert(Near((SkewSymmetric(vector) * vector).norm(), 0.0, 1e-12));
  const Eigen::Quaterniond yaw_quaternion = ExpMapSo3(Eigen::Vector3d(0.0, 0.0, 0.1));
  assert(Near(LogMapSo3(yaw_quaternion).z(), 0.1, 1e-12));
  const EulerAngles euler{DegToRad(10.0), DegToRad(20.0), DegToRad(30.0)};
  const EulerAngles euler_round_trip = QuaternionToEuler(EulerToQuaternion(euler));
  assert(Near(euler_round_trip.roll_rad, euler.roll_rad, 1e-12));
  assert(Near(euler_round_trip.pitch_rad, euler.pitch_rad, 1e-12));
  assert(Near(euler_round_trip.yaw_rad, euler.yaw_rad, 1e-12));

  const auto parsed_sentence = falconguide::io::nmea::ParseSentence("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47");
  assert(parsed_sentence);
  assert(parsed_sentence->formatter == "GGA");
  const auto parsed_gga = falconguide::io::nmea::ParseGga(*parsed_sentence);
  assert(parsed_gga);
  const Lla parsed_gga_lla = EcefToLla(parsed_gga->position_ecef_m);
  assert(Near(RadToDeg(parsed_gga_lla.latitude_rad), 48.1173, 1e-4));
  assert(Near(RadToDeg(parsed_gga_lla.longitude_rad), 11.5166667, 1e-4));

  ImuMeasurement imu;
  imu.specific_force_mps2 = Vec3<ImuFrame>(0.0, 0.0, -9.80665);
  imu.angular_rate_radps = Vec3<ImuFrame>(0.01, 0.02, 0.03);
  imu.timestamp.has_steady = true;
  imu.timestamp.steady = MonotonicTime(std::chrono::nanoseconds(10));
  assert(imu.validity == MeasurementValidity::Valid);
  assert(IsValid(imu));
  ImuMeasurement later_imu = imu;
  later_imu.timestamp.steady = MonotonicTime(std::chrono::nanoseconds(20));
  assert(IsOutOfOrder(later_imu, imu));

  TimeOrderedBuffer<ImuMeasurement> imu_buffer(2);
  assert(imu_buffer.Insert(later_imu) == BufferInsertResult::Inserted);
  assert(imu_buffer.Insert(imu) == BufferInsertResult::Inserted);
  assert(imu_buffer.Oldest()->timestamp.steady.nanoseconds_since_epoch().count() == 10);
  ImuMeasurement latest_imu = imu;
  latest_imu.timestamp.steady = MonotonicTime(std::chrono::nanoseconds(30));
  assert(imu_buffer.Insert(latest_imu) == BufferInsertResult::InsertedAndDroppedOldest);
  assert(imu_buffer.size() == 2);
  const std::vector<ImuMeasurement> popped_imu = imu_buffer.PopUntil(later_imu.timestamp);
  assert(popped_imu.size() == 1);

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

  NavigationState state;
  state.status = NavigationStatus::Nominal;
  state.mode = EstimatorMode::MultiSensorFusion;
  state.quality.initialized = true;
  state.quality.position_accuracy_m = 0.8;
  state.sensors.imu.health = SensorHealth::Healthy;
  state.sensors.imu.used_in_solution = true;
  state.sensors.gnss.health = SensorHealth::Rejected;
  state.sensors.gnss.innovation_norm = 12.0;
  state.sensors.visual_odometry.health = SensorHealth::Healthy;
  state.sensors.visual_odometry.used_in_solution = true;
  assert(state.status == NavigationStatus::Nominal);
  assert(state.quality.initialized);
  assert(state.sensors.gnss.health == SensorHealth::Rejected);
  state.position_ecef_m = LlaToEcef(Lla{DegToRad(48.1173), DegToRad(11.5166667), 545.4});
  const std::string gga_output = falconguide::io::nmea::WriteGga(state, "123519");
  assert(falconguide::io::nmea::ParseSentence(gga_output));

  SensorMeasurement measurement = imu;
  assert(std::holds_alternative<ImuMeasurement>(measurement));
  assert(falconguide::estimation::IsValid(measurement));
  assert(falconguide::estimation::GetTimestamp(measurement).has_steady);
  measurement = GnssSolution{};
  assert(std::holds_alternative<GnssSolution>(measurement));

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

  // ── diagnostics: ToString ──────────────────────────────────────────────────
  assert(falconguide::core::ToString(NavigationStatus::Nominal)        == "Nominal");
  assert(falconguide::core::ToString(NavigationStatus::DeadReckoning)  == "DeadReckoning");
  assert(falconguide::core::ToString(NavigationStatus::Fault)          == "Fault");

  assert(falconguide::core::ToString(EstimatorMode::VisualInertialGnss) == "VisualInertialGnss");
  assert(falconguide::core::ToString(EstimatorMode::InertialOnly)       == "InertialOnly");

  assert(falconguide::core::ToString(SensorHealth::Healthy)  == "Healthy");
  assert(falconguide::core::ToString(SensorHealth::Stale)    == "Stale");
  assert(falconguide::core::ToString(SensorHealth::Rejected) == "Rejected");

  assert(falconguide::core::ToString(MeasurementValidity::Valid)      == "Valid");
  assert(falconguide::core::ToString(MeasurementValidity::OutOfOrder) == "OutOfOrder");

  assert(falconguide::core::ToString(GnssFixType::RtkFixed)               == "RtkFixed");
  assert(falconguide::core::ToString(GnssFixType::PrecisePointPositioning) == "PrecisePointPositioning");
  assert(falconguide::core::ToString(GnssFixType::NoFix)                  == "NoFix");

  assert(falconguide::core::ToString(EstimatorBackend::Ekf)               == "EKF");
  assert(falconguide::core::ToString(EstimatorBackend::GtsamFactorGraph)   == "GtsamFactorGraph");
  assert(falconguide::core::ToString(EstimatorUpdateResult::Accepted)      == "Accepted");
  assert(falconguide::core::ToString(EstimatorUpdateResult::BackendError)  == "BackendError");

  // ── IMeasurementReader / IMeasurementWriter: compile-time interface check ──
  static_assert(std::is_abstract_v<falconguide::io::IMeasurementReader>);
  static_assert(std::is_abstract_v<falconguide::io::IMeasurementWriter>);

  // ReaderCapabilities bitmask
  using RC = falconguide::io::ReaderCapabilities;
  const RC caps = RC::Imu | RC::Gnss | RC::Camera;
  assert(falconguide::io::HasCapability(caps, RC::Imu));
  assert(falconguide::io::HasCapability(caps, RC::Gnss));
  assert(falconguide::io::HasCapability(caps, RC::Camera));
  assert(!falconguide::io::HasCapability(caps, RC::Barometer));

  // ── NMEA RMC round-trip ────────────────────────────────────────────────────
  NavigationState rmc_state;
  rmc_state.status = NavigationStatus::Nominal;
  const Lla rmc_lla{DegToRad(51.5), DegToRad(-0.12), 30.0};
  rmc_state.position_ecef_m = LlaToEcef(rmc_lla);
  rmc_state.velocity_enu_mps = Vec3<EnuFrame>(5.0, 10.0, 0.0);  // east=5, north=10

  const std::string rmc_sentence = falconguide::io::nmea::WriteRmc(rmc_state, "120000.00", "160526");
  assert(!rmc_sentence.empty());
  assert(rmc_sentence.front() == '$');

  const auto rmc_parsed = falconguide::io::nmea::ParseRmcLine(rmc_sentence);
  assert(rmc_parsed.has_value());
  assert(rmc_parsed->fix_type == GnssFixType::Single);
  assert(rmc_parsed->validity == MeasurementValidity::Valid);

  // Void RMC (no fix) should not parse to a solution.
  const std::string rmc_void = "$FGRMC,,V,,,,,,,,,,N*XX";  // invalid checksum, just structural
  assert(!falconguide::io::nmea::ParseRmcLine(rmc_void).has_value());

  // ── NMEA VTG round-trip ────────────────────────────────────────────────────
  NavigationState vtg_state;
  vtg_state.status = NavigationStatus::Nominal;
  vtg_state.velocity_enu_mps = Vec3<EnuFrame>(0.0, 10.0, 0.0);  // due north, 10 m/s

  const std::string vtg_sentence = falconguide::io::nmea::WriteVtg(vtg_state);
  assert(!vtg_sentence.empty());

  const auto vtg_parsed = falconguide::io::nmea::ParseVtgLine(vtg_sentence);
  assert(vtg_parsed.has_value());
  // Due north → course = 0°
  assert(Near(vtg_parsed->course_true_deg, 0.0, 0.01));
  // 10 m/s ≈ 19.438 knots
  assert(Near(vtg_parsed->speed_knots, 10.0 / 0.514444, 0.01));

  return 0;
}
