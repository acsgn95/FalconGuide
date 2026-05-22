#pragma once

/**
 * @file ukf_state.hpp
 * @brief Unscented Kalman Filter state, sigma-point, and noise types.
 */

#include "falconguide/core/frames.hpp"
#include "falconguide/core/time.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace falconguide::estimation::ukf {

// ── UKF State Size
// ────────────────────────────────────────────────────────────
//
// Same 15-element error state as the EKF:
//   [0:2]  position ENU (m)
//   [3:5]  velocity ENU (m/s)
//   [6:8]  attitude error (rotation vector, body frame)
//   [9:11] accelerometer bias (m/s²)
//  [12:14] gyroscope bias (rad/s)
//
// Unlike the EKF, the UKF does not linearise the process / measurement models.
// It propagates a set of carefully chosen "sigma points" through the exact
// (nonlinear) functions instead, recovering mean and covariance from the
// result.
//
static constexpr int kStateSize = 15; ///< UKF state-vector dimension.

using StateVector =
    Eigen::Matrix<double, kStateSize, 1>; ///< Fixed-size UKF state vector.
using StateCovariance =
    Eigen::Matrix<double, kStateSize,
                  kStateSize>; ///< Fixed-size UKF covariance matrix.

// Number of sigma points for a kStateSize-dimensional state.
// Merwe scaled sigma point rule: 2n + 1.
static constexpr int kNumSigmaPoints =
    2 * kStateSize + 1; ///< Number of Merwe sigma points.

using SigmaMatrix =
    Eigen::Matrix<double, kStateSize,
                  kNumSigmaPoints>; ///< Matrix containing all sigma points.

// ── Merwe Scaled Sigma Point Parameters ──────────────────────────────────────
//
// α  controls the spread of sigma points around the mean (typically 1e-3 to 1).
// β  incorporates prior knowledge of the distribution (2 is optimal for
// Gaussian). κ  secondary scaling parameter (0 or 3 − n are common choices).
//
struct MerweSigmaParams {
  double alpha{1e-3}; ///< Sigma-point spread.
  double beta{
      2.0}; ///< Prior distribution parameter; 2 is optimal for Gaussian priors.
  double kappa{0.0}; ///< Secondary scaling parameter.
};

// Derived weights for mean and covariance recovery from sigma points.
/// @brief Mean and covariance weights for Merwe sigma points.
struct SigmaWeights {
  Eigen::Matrix<double, kNumSigmaPoints, 1>
      mean_weights; ///< Weights used for mean recovery.
  Eigen::Matrix<double, kNumSigmaPoints, 1>
      cov_weights; ///< Weights used for covariance recovery.

  /// @brief Computes sigma-point weights from Merwe parameters.
  static SigmaWeights Compute(const MerweSigmaParams &params) noexcept;
};

// ── Nominal State
// ─────────────────────────────────────────────────────────────
//
// The UKF operates directly on the mean state vector (no error-state split
// needed for the translational and velocity components). Attitude is handled
// as a quaternion in the nominal state and mapped to/from a rotation vector
// for the sigma point arithmetic.
//
struct NominalState {
  core::Timestamp timestamp; ///< State timestamp.

  core::Vec3<core::EnuFrame> position_enu_m;   ///< ENU position mean.
  core::Vec3<core::EnuFrame> velocity_enu_mps; ///< ENU velocity mean.
  Eigen::Quaterniond orientation_body_to_enu{
      Eigen::Quaterniond::Identity()};        ///< Body-to-ENU attitude mean.
  core::Vec3<core::ImuFrame> accel_bias_mps2; ///< Accelerometer bias mean.
  core::Vec3<core::ImuFrame> gyro_bias_radps; ///< Gyroscope bias mean.
};

// ── IMU Noise Model
// ─────────────────────────────────────────────────────────── (Same definition
// as EKF — kept separate so backends remain independent.)

struct ImuNoiseModel {
  double accel_noise_density_mps2_per_sqrthz{
      3.0e-3}; ///< Accelerometer white-noise density.
  double gyro_noise_density_radps_per_sqrthz{
      1.5e-4}; ///< Gyroscope white-noise density.
  double accel_random_walk_mps3_per_sqrthz{
      3.0e-5}; ///< Accelerometer bias random walk.
  double gyro_random_walk_radps2_per_sqrthz{
      2.0e-6}; ///< Gyroscope bias random walk.
};

// ── Full UKF State
// ────────────────────────────────────────────────────────────

struct UkfState {
  NominalState nominal; ///< Current nominal mean state.
  StateCovariance covariance{StateCovariance::Identity() *
                             1e-4}; ///< Current state covariance.
  SigmaMatrix sigma_points{
      SigmaMatrix::Zero()}; ///< Most recently generated sigma points.
};

} // namespace falconguide::estimation::ukf
