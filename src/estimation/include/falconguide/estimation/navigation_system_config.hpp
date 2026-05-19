#pragma once

#include "falconguide/estimation/backends/ekf/ekf_state.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/gnss_loosely_coupled.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/gnss_tightly_coupled.hpp"

namespace falconguide::estimation {

// ── Per-sensor config blocks ──────────────────────────────────────────────────
//
// Each sensor family has its own struct.  Disabled sensors are zero-cost:
// no measurement model is registered and no state segment is allocated.
//
// To add a new sensor type, add a new config struct here and handle it in
// NavigationSystem's factory logic.  Nothing else needs to change.
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
  bool   enabled{false};
  double altitude_sigma_m{0.5};         // 1-sigma altitude noise (m)
  double innovation_gate_sigma{3.0};    // 0 = no gate
};

// ─────────────────────────────────────────────────────────────────────────────

struct StarTrackerSensorConfig {
  bool   enabled{false};
  // Horizontal position uncertainty derived from lat/lon fix (m, 1-sigma).
  // Set large (e.g. 500.0) if you want the star tracker to update heading only.
  double position_sigma_m{50.0};
  double yaw_sigma_rad{5e-3};           // heading uncertainty (rad, 1-sigma)
  // Pitch / roll updates are applied only when the star tracker provides them.
  double pitch_sigma_rad{1e-2};
  double roll_sigma_rad{1e-2};
  double innovation_gate_sigma{3.0};
};

// ─────────────────────────────────────────────────────────────────────────────

struct MagnetometerSensorConfig {
  bool   enabled{false};
  // Not yet implemented — reserved for future heading update model.
};

// ─────────────────────────────────────────────────────────────────────────────

struct DvlSensorConfig {
  bool   enabled{false};
  // Not yet implemented — reserved for future DVL velocity update model.
};

// ─────────────────────────────────────────────────────────────────────────────

struct AirspeedSensorConfig {
  bool   enabled{false};
  // Not yet implemented.
};

// ── NavigationSystemConfig ────────────────────────────────────────────────────
//
// Complete system configuration.  Write one instance per hardware platform;
// pass it to NavigationSystem and the factory does the rest.
//
//   Example (custom UAV board with u-blox F9P + ICM-42688 + MS5611):
//
//     NavigationSystemConfig cfg;
//     cfg.imu.noise.accel_noise_density_mps2_per_sqrthz = 0.0028;
//     cfg.imu.noise.gyro_noise_density_radps_per_sqrthz  = 8.7e-5;
//     cfg.gnss.enabled = true;
//     cfg.gnss.mode    = GnssIntegrationMode::LooselyCoupled;
//     cfg.barometer.enabled         = true;
//     cfg.barometer.altitude_sigma_m = 0.3;
//     NavigationSystem nav(cfg);
//
struct NavigationSystemConfig {
  ImuSensorConfig         imu;
  GnssSensorConfig        gnss;
  BarometerSensorConfig   barometer;
  StarTrackerSensorConfig star_tracker;
  MagnetometerSensorConfig magnetometer;
  DvlSensorConfig         dvl;
  AirspeedSensorConfig    airspeed;

  // EKF engine tuning
  double dead_reckoning_threshold_s{5.0};
  double min_imu_dt_s{1e-6};
  double max_imu_dt_s{0.05};
};

}  // namespace falconguide::estimation
