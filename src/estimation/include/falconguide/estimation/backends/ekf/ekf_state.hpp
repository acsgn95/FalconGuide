#pragma once

#include "falconguide/core/frames.hpp"
#include "falconguide/core/time.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace falconguide::estimation::ekf {

// 15-element error-state vector:
//   [0:2]  δp  — position error in ENU (m)
//   [3:5]  δv  — velocity error in ENU (m/s)
//   [6:8]  δθ  — attitude error as rotation vector in body frame (rad)
//   [9:11] δba — accelerometer bias error in body frame (m/s²)
//  [12:14] δbg — gyroscope bias error in body frame (rad/s)
static constexpr int kStateSize = 15;

using ErrorState    = Eigen::Matrix<double, kStateSize, 1>;
using ErrorCovariance = Eigen::Matrix<double, kStateSize, kStateSize>;

// Nominal (reference) state — the "large" state propagated between updates.
struct NominalState {
  core::Timestamp timestamp;

  // Position and velocity in ENU local tangent plane (origin set at first GNSS fix).
  core::Vec3<core::EnuFrame> position_enu_m;
  core::Vec3<core::EnuFrame> velocity_enu_mps;

  // Body-to-ENU rotation (quaternion, always normalized).
  Eigen::Quaterniond orientation_body_to_enu{Eigen::Quaterniond::Identity()};

  // IMU biases expressed in body frame.
  core::Vec3<core::ImuFrame> accel_bias_mps2;
  core::Vec3<core::ImuFrame> gyro_bias_radps;
};

// IMU noise model — all as continuous-time densities.
struct ImuNoiseModel {
  double accel_noise_density_mps2_per_sqrthz{3.0e-3};   // σ_a
  double gyro_noise_density_radps_per_sqrthz{1.5e-4};    // σ_ω
  double accel_random_walk_mps3_per_sqrthz{3.0e-5};      // σ_ba
  double gyro_random_walk_radps2_per_sqrthz{2.0e-6};     // σ_bω
};

// Full EKF state: nominal + error covariance.
struct EkfState {
  NominalState nominal;
  ErrorCovariance covariance{ErrorCovariance::Identity() * 1e-4};
};

}  // namespace falconguide::estimation::ekf
