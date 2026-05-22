#pragma once

#include "falconguide/estimation/backends/ekf/ekf_state.hpp"
#include "falconguide/estimation/backends/ukf/ukf_state.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/aiding_solution.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/barometer.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/airspeed.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/dvl.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/echo_sounder.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/external_odometry.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/external_pose.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/external_velocity.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/gnss_loosely_coupled.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/gnss_tightly_coupled.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/magnetometer.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/optical_flow.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/radar_altimeter.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/range_finder.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/star_tracker.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/wheel_odometry.hpp"

namespace falconguide::estimation {

enum class EstimatorBackendChoice { Ekf, Ukf, Ceres, Gtsam };

// ── Per-sensor config blocks ──────────────────────────────────────────────────
//
// Each block has an enabled flag plus the relevant model options.  Disabled
// sensors are zero-cost: no model is registered and no state segment is added.
//
// Pass NavigationSystemConfig to NavigationSystem and the factory wires
// everything automatically.
//

struct ImuSensorConfig {
  ekf::ImuNoiseModel noise;
};

// ─────────────────────────────────────────────────────────────────────────────

enum class GnssIntegrationMode {
  LooselyCoupled,   // fuses GnssSolution (pos + vel)
  TightlyCoupled,   // fuses GnssTightlyCoupledEpoch (raw pseudorange + Doppler)
};

struct GnssSensorConfig {
  bool enabled{false};
  GnssIntegrationMode mode{GnssIntegrationMode::LooselyCoupled};
  ekf::GnssLooselyCoupledOptions loosely_coupled;
  ekf::GnssTightlyCoupledOptions tightly_coupled;
};

// ─────────────────────────────────────────────────────────────────────────────

struct BarometerSensorConfig {
  bool enabled{false};
  ekf::BarometerOptions options;
};

// ─────────────────────────────────────────────────────────────────────────────

struct MagnetometerSensorConfig {
  bool enabled{false};
  ekf::MagnetometerOptions options;
};

// ─────────────────────────────────────────────────────────────────────────────

struct StarTrackerSensorConfig {
  bool enabled{false};
  ekf::StarTrackerOptions options;
};

// ─────────────────────────────────────────────────────────────────────────────

struct WheelOdometrySensorConfig {
  bool enabled{false};
  ekf::WheelOdometryOptions options;
};

// ─────────────────────────────────────────────────────────────────────────────

struct RadarAltimeterSensorConfig {
  bool enabled{false};
  ekf::RadarAltimeterOptions options;
};

// ─────────────────────────────────────────────────────────────────────────────

struct RangeFinderSensorConfig {
  bool enabled{false};
  ekf::RangeFinderOptions options;
};

// ─────────────────────────────────────────────────────────────────────────────

struct OpticalFlowSensorConfig {
  bool enabled{false};
  ekf::OpticalFlowOptions options;
};

// ─────────────────────────────────────────────────────────────────────────────

struct DvlSensorConfig {
  bool enabled{false};
  ekf::DvlOptions options;
};

// ─────────────────────────────────────────────────────────────────────────────

struct EchoSounderSensorConfig {
  bool enabled{false};
  ekf::EchoSounderOptions options;
};

// ─────────────────────────────────────────────────────────────────────────────

struct AirspeedSensorConfig {
  bool enabled{false};
  ekf::AirspeedOptions options;
};

// ─────────────────────────────────────────────────────────────────────────────

struct ExternalPoseSensorConfig {
  bool enabled{false};
  ekf::ExternalPoseOptions options;
};

// ─────────────────────────────────────────────────────────────────────────────

struct ExternalVelocitySensorConfig {
  bool enabled{false};
  ekf::ExternalVelocityOptions options;
};

// ─────────────────────────────────────────────────────────────────────────────

struct ExternalOdometrySensorConfig {
  bool enabled{false};
  ekf::ExternalOdometryOptions options;
};

// ─────────────────────────────────────────────────────────────────────────────

struct AidingSolutionSensorConfig {
  bool enabled{false};
  ekf::AidingSolutionOptions options;
};

// ── NavigationSystemConfig ────────────────────────────────────────────────────
//
// Complete system configuration.  Write one instance per hardware platform and
// pass it to NavigationSystem.
//
// Example (UAV with u-blox F9P + ICM-42688 + MS5611 + optical flow):
//
//   NavigationSystemConfig cfg;
//   cfg.imu.noise.accel_noise_density_mps2_per_sqrthz = 0.0028;
//   cfg.gnss.enabled = true;
//   cfg.barometer.enabled = true;
//   cfg.optical_flow.enabled = true;
//   NavigationSystem nav(cfg);
//
struct NavigationSystemConfig {
  ImuSensorConfig            imu;
  GnssSensorConfig           gnss;
  BarometerSensorConfig      barometer;
  MagnetometerSensorConfig   magnetometer;
  StarTrackerSensorConfig    star_tracker;
  WheelOdometrySensorConfig  wheel_odometry;
  RadarAltimeterSensorConfig radar_altimeter;
  RangeFinderSensorConfig    range_finder;
  OpticalFlowSensorConfig    optical_flow;
  DvlSensorConfig            dvl;
  EchoSounderSensorConfig    echo_sounder;
  AirspeedSensorConfig       airspeed;
  ExternalPoseSensorConfig   external_pose;
  ExternalVelocitySensorConfig  external_velocity;
  ExternalOdometrySensorConfig  external_odometry;
  AidingSolutionSensorConfig    aiding_solution;

  // Backend selection
  EstimatorBackendChoice backend{EstimatorBackendChoice::Ekf};

  // UKF-only: Merwe scaled sigma point parameters.
  // Ignored when backend == Ekf.
  ukf::MerweSigmaParams ukf_sigma_params;

  // EKF engine tuning
  double dead_reckoning_threshold_s{5.0};
  double min_imu_dt_s{1e-6};
  double max_imu_dt_s{0.05};
};

}  // namespace falconguide::estimation
