#pragma once

#include "falconguide/core/frames.hpp"
#include "falconguide/core/time.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace falconguide::estimation::ukf {

// ── UKF State Size ────────────────────────────────────────────────────────────
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
// (nonlinear) functions instead, recovering mean and covariance from the result.
//
static constexpr int kStateSize = 15;

using StateVector   = Eigen::Matrix<double, kStateSize, 1>;
using StateCovariance = Eigen::Matrix<double, kStateSize, kStateSize>;

// Number of sigma points for a kStateSize-dimensional state.
// Merwe scaled sigma point rule: 2n + 1.
static constexpr int kNumSigmaPoints = 2 * kStateSize + 1;

using SigmaMatrix = Eigen::Matrix<double, kStateSize, kNumSigmaPoints>;

// ── Merwe Scaled Sigma Point Parameters ──────────────────────────────────────
//
// α  controls the spread of sigma points around the mean (typically 1e-3 to 1).
// β  incorporates prior knowledge of the distribution (2 is optimal for Gaussian).
// κ  secondary scaling parameter (0 or 3 − n are common choices).
//
struct MerweSigmaParams {
  double alpha{1e-3};
  double beta{2.0};
  double kappa{0.0};
};

// Derived weights for mean and covariance recovery from sigma points.
struct SigmaWeights {
  Eigen::Matrix<double, kNumSigmaPoints, 1> mean_weights;
  Eigen::Matrix<double, kNumSigmaPoints, 1> cov_weights;

  static SigmaWeights Compute(const MerweSigmaParams& params) noexcept;
};

// ── Nominal State ─────────────────────────────────────────────────────────────
//
// The UKF operates directly on the mean state vector (no error-state split
// needed for the translational and velocity components). Attitude is handled
// as a quaternion in the nominal state and mapped to/from a rotation vector
// for the sigma point arithmetic.
//
struct NominalState {
  core::Timestamp timestamp;

  core::Vec3<core::EnuFrame>  position_enu_m;
  core::Vec3<core::EnuFrame>  velocity_enu_mps;
  Eigen::Quaterniond          orientation_body_to_enu{Eigen::Quaterniond::Identity()};
  core::Vec3<core::ImuFrame>  accel_bias_mps2;
  core::Vec3<core::ImuFrame>  gyro_bias_radps;
};

// ── IMU Noise Model ───────────────────────────────────────────────────────────
// (Same definition as EKF — kept separate so backends remain independent.)

struct ImuNoiseModel {
  double accel_noise_density_mps2_per_sqrthz{3.0e-3};
  double gyro_noise_density_radps_per_sqrthz{1.5e-4};
  double accel_random_walk_mps3_per_sqrthz{3.0e-5};
  double gyro_random_walk_radps2_per_sqrthz{2.0e-6};
};

// ── Full UKF State ────────────────────────────────────────────────────────────

struct UkfState {
  NominalState    nominal;
  StateCovariance covariance{StateCovariance::Identity() * 1e-4};
  SigmaMatrix     sigma_points{SigmaMatrix::Zero()};
};

}  // namespace falconguide::estimation::ukf
