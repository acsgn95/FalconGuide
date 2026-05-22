#pragma once

#include "falconguide/core/frames.hpp"
#include "falconguide/core/time.hpp"
#include "falconguide/estimation/backends/ekf/state_layout.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <optional>

namespace falconguide::estimation::ekf {

// ── NominalState ──────────────────────────────────────────────────────────────
//
// The "large" reference trajectory propagated between updates.
// Optional fields are populated only when the corresponding StateSegmentId is
// active in the layout (e.g. gnss_clock is non-null only when GnssClock is
// registered, which is required for tightly coupled GNSS).
//
struct NominalState {
  core::Timestamp timestamp;

  core::Vec3<core::EnuFrame> position_enu_m;
  core::Vec3<core::EnuFrame> velocity_enu_mps;
  Eigen::Quaterniond orientation_body_to_enu{Eigen::Quaterniond::Identity()};

  core::Vec3<core::ImuFrame> accel_bias_mps2;
  core::Vec3<core::ImuFrame> gyro_bias_radps;

  // [0]: receiver clock bias (m), [1]: clock drift (m/s).
  // Populated only when StateSegmentId::GnssClock is active.
  std::optional<Eigen::Vector2d> gnss_clock;
};

// ── ImuNoiseModel ─────────────────────────────────────────────────────────────
//
// Continuous-time spectral densities.  Units match the standard IMU datasheet
// notation: noise density (random noise) and random walk (bias instability).
//
struct ImuNoiseModel {
  double accel_noise_density_mps2_per_sqrthz{3.0e-3};   // σ_a
  double gyro_noise_density_radps_per_sqrthz{1.5e-4};    // σ_ω
  double accel_random_walk_mps3_per_sqrthz{3.0e-5};      // σ_ba
  double gyro_random_walk_radps2_per_sqrthz{2.0e-6};     // σ_bω
};

// ── EkfState ──────────────────────────────────────────────────────────────────
//
// Complete dynamic EKF state: nominal trajectory + error-state distribution.
//
// The error_state vector is always near zero — it is reset to zero after each
// measurement update injects the correction into the nominal state.
// The covariance grows during IMU propagation and shrinks during updates.
//
struct EkfState {
  NominalState    nominal;
  Eigen::VectorXd error_state;   // zeroed after every inject-and-reset
  Eigen::MatrixXd covariance;
  StateLayout     layout;
};

}  // namespace falconguide::estimation::ekf
