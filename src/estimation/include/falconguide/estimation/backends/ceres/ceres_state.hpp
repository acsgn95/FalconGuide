#pragma once

#include "falconguide/core/frames.hpp"
#include "falconguide/core/time.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstddef>
#include <vector>

namespace falconguide::estimation::ceres_backend {

// ── Keyframe ──────────────────────────────────────────────────────────────────
//
// One node in the sliding window. Each keyframe holds a pose, velocity, and
// IMU biases. These become Ceres parameter blocks during optimisation.
//
// Parameter block layout (for Ceres):
//   pose_block[7]  = [px, py, pz, qx, qy, qz, qw]   (ENU position + body→ENU quat)
//   vel_block[3]   = [vx, vy, vz]                    (ENU velocity)
//   bias_block[6]  = [bax, bay, baz, bgx, bgy, bgz]  (acc + gyro bias, body frame)
//
struct Keyframe {
  core::Timestamp timestamp;

  // ENU position and body→ENU orientation.
  core::Vec3<core::EnuFrame>   position_enu_m;
  Eigen::Quaterniond           orientation_body_to_enu{Eigen::Quaterniond::Identity()};

  // ENU velocity.
  core::Vec3<core::EnuFrame>   velocity_enu_mps;

  // IMU biases at this keyframe.
  core::Vec3<core::ImuFrame>   accel_bias_mps2;
  core::Vec3<core::ImuFrame>   gyro_bias_radps;

  // Is this keyframe marginalised (kept as linearisation point only)?
  bool marginalised{false};
};

// ── IMU Preintegration ────────────────────────────────────────────────────────
//
// Stores the result of integrating raw IMU measurements between two keyframes.
// This is the "IMU factor" input for Ceres.
//
// Based on the Forster et al. manifold preintegration theory (TRO 2017).
//
struct ImuPreintegration {
  core::Timestamp start_timestamp;
  core::Timestamp end_timestamp;

  // Preintegrated delta measurements.
  Eigen::Vector3d   delta_p{Eigen::Vector3d::Zero()};   // position increment
  Eigen::Quaterniond delta_q{Eigen::Quaterniond::Identity()};  // rotation increment
  Eigen::Vector3d   delta_v{Eigen::Vector3d::Zero()};   // velocity increment

  // Covariance of the preintegrated delta [9×9: dp, dq, dv].
  Eigen::Matrix<double, 9, 9> covariance{Eigen::Matrix<double, 9, 9>::Zero()};

  // Jacobians w.r.t. biases at linearisation point (for first-order correction).
  Eigen::Matrix<double, 9, 3> jacobian_dp_dba{Eigen::Matrix<double, 9, 3>::Zero()};
  Eigen::Matrix<double, 9, 3> jacobian_dp_dbg{Eigen::Matrix<double, 9, 3>::Zero()};

  // Bias linearisation point used during preintegration.
  core::Vec3<core::ImuFrame> linearisation_accel_bias_mps2;
  core::Vec3<core::ImuFrame> linearisation_gyro_bias_radps;

  double integration_time_s{0.0};
  std::size_t num_samples{0};
};

// ── Marginalisation Prior ─────────────────────────────────────────────────────
//
// Dense prior produced by marginalising old keyframes out of the window
// (Schur complement). Constrains the remaining states at the window boundary.
//
struct MarginalisationPrior {
  // Jacobian and residual of the linearised prior factor.
  Eigen::MatrixXd jacobian;   // [n_residuals × n_params]
  Eigen::VectorXd residual;   // [n_residuals]

  // Ordering of parameter blocks this prior references.
  std::vector<core::Timestamp> keyframe_timestamps;

  bool is_valid{false};
};

// ── Sliding Window State ──────────────────────────────────────────────────────

struct SlidingWindowState {
  std::vector<Keyframe>          keyframes;         // oldest → newest
  std::vector<ImuPreintegration> imu_factors;       // one per consecutive keyframe pair
  MarginalisationPrior           marginalisation_prior;
};

}  // namespace falconguide::estimation::ceres_backend
