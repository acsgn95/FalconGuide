#pragma once

#ifdef FALCONGUIDE_HAVE_CERES

#include "falconguide/estimation/backends/ceres/ceres_state.hpp"

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace falconguide::estimation::ceres_backend {

static constexpr double kGravityMps2 = 9.80665;
static const Eigen::Vector3d kGravityEnu{0.0, 0.0, -kGravityMps2};

// ── Helpers ───────────────────────────────────────────────────────────────────

// Quaternion exponential map (template for Ceres Jet support)
template <typename T>
static Eigen::Quaternion<T> QuatExp(const Eigen::Matrix<T, 3, 1>& phi) {
  const T angle = phi.norm();
  if (angle < T(1e-10))
    return {T(1), phi(0) * T(0.5), phi(1) * T(0.5), phi(2) * T(0.5)};
  const T s = ceres::sin(T(0.5) * angle) / angle;
  return {ceres::cos(T(0.5) * angle), s * phi(0), s * phi(1), s * phi(2)};
}

// 2*[q]_xyz of a near-identity quaternion (log map approximation for residual)
template <typename T>
static Eigen::Matrix<T, 3, 1> QuatResidual(const Eigen::Quaternion<T>& q) {
  // q = [w, x, y, z]; if w < 0 flip to ensure shortest path
  const T w = (q.w() > T(0)) ? q.w() : -q.w();
  const T x = (q.w() > T(0)) ? q.x() : -q.x();
  const T y = (q.w() > T(0)) ? q.y() : -q.y();
  const T z = (q.w() > T(0)) ? q.z() : -q.z();
  return {T(2) * x, T(2) * y, T(2) * z};
}

// ── IMU Preintegration Cost ───────────────────────────────────────────────────
//
// Residual (9-DOF): [r_p(3), r_q(3), r_v(3)]
//
// Parameter blocks:
//   pose_i[7]  = [px, py, pz, qx, qy, qz, qw]  (position + body→ENU quat)
//   vel_i[3]   = [vx, vy, vz]
//   bias_i[6]  = [bax, bay, baz, bgx, bgy, bgz]
//   pose_j[7], vel_j[3], bias_j[6]   (same layout for keyframe j)
//
struct ImuPreintegrationCost {
  explicit ImuPreintegrationCost(ImuPreintegration preint)
      : preint_(std::move(preint)) {}

  template <typename T>
  bool operator()(const T* pose_i, const T* vel_i,  const T* bias_i,
                  const T* pose_j, const T* vel_j,  const T* bias_j,
                  T* residual) const {
    // Unpack pose_i
    Eigen::Matrix<T, 3, 1> p_i(pose_i[0], pose_i[1], pose_i[2]);
    Eigen::Quaternion<T>   q_i(pose_i[6], pose_i[3], pose_i[4], pose_i[5]);  // w,x,y,z
    q_i.normalize();
    const Eigen::Matrix<T, 3, 3> R_i = q_i.toRotationMatrix();

    // Unpack pose_j
    Eigen::Matrix<T, 3, 1> p_j(pose_j[0], pose_j[1], pose_j[2]);
    Eigen::Quaternion<T>   q_j(pose_j[6], pose_j[3], pose_j[4], pose_j[5]);
    q_j.normalize();

    // Velocities
    Eigen::Matrix<T, 3, 1> v_i(vel_i[0], vel_i[1], vel_i[2]);
    Eigen::Matrix<T, 3, 1> v_j(vel_j[0], vel_j[1], vel_j[2]);

    // Bias correction delta (first-order)
    Eigen::Matrix<T, 3, 1> dba(bias_i[0] - T(preint_.linearisation_accel_bias_mps2.x()),
                                bias_i[1] - T(preint_.linearisation_accel_bias_mps2.y()),
                                bias_i[2] - T(preint_.linearisation_accel_bias_mps2.z()));
    Eigen::Matrix<T, 3, 1> dbg(bias_i[3] - T(preint_.linearisation_gyro_bias_radps.x()),
                                bias_i[4] - T(preint_.linearisation_gyro_bias_radps.y()),
                                bias_i[5] - T(preint_.linearisation_gyro_bias_radps.z()));

    // Corrected preintegrated values (first-order bias correction)
    const auto Jdp_dba = preint_.jacobian_dp_dba.template cast<T>();
    const auto Jdp_dbg = preint_.jacobian_dp_dbg.template cast<T>();

    Eigen::Matrix<T, 3, 1> dp_corr = preint_.delta_p.template cast<T>()
        + Jdp_dba.block(0, 0, 3, 3) * dba
        + Jdp_dbg.block(0, 0, 3, 3) * dbg;

    Eigen::Matrix<T, 3, 1> dq_corr_axis = preint_.jacobian_dp_dba.block(3, 0, 3, 3).template cast<T>() * dbg;
    Eigen::Quaternion<T>   dq_corr      = QuatExp(dq_corr_axis);

    Eigen::Matrix<T, 3, 1> dv_corr = preint_.delta_v.template cast<T>()
        + Jdp_dba.block(6, 0, 3, 3) * dba
        + Jdp_dbg.block(6, 0, 3, 3) * dbg;

    const T dt  = T(preint_.integration_time_s);
    const Eigen::Matrix<T, 3, 1> g_enu(T(0), T(0), T(-kGravityMps2));

    // Position residual: R_i^T*(p_j - p_i - v_i*dt - 0.5*g*dt²) - Δp_corr
    Eigen::Matrix<T, 3, 1> r_p =
        R_i.transpose() * (p_j - p_i - v_i * dt - T(0.5) * g_enu * dt * dt) - dp_corr;

    // Rotation residual: 2*[ΔqCorr^{-1} ⊗ q_i^{-1} ⊗ q_j]_xyz
    Eigen::Quaternion<T>   dq_hat(T(preint_.delta_q.w()), T(preint_.delta_q.x()),
                                  T(preint_.delta_q.y()), T(preint_.delta_q.z()));
    Eigen::Quaternion<T>   q_err = (dq_hat * dq_corr).conjugate() * (q_i.conjugate() * q_j);
    Eigen::Matrix<T, 3, 1> r_q  = QuatResidual(q_err);

    // Velocity residual: R_i^T*(v_j - v_i - g*dt) - Δv_corr
    Eigen::Matrix<T, 3, 1> r_v =
        R_i.transpose() * (v_j - v_i - g_enu * dt) - dv_corr;

    // Apply information matrix (Cholesky of preint covariance inverse)
    // Simplified: isotropic weighting using trace for now
    const double info_scale = 1.0 / std::max(1e-6, std::sqrt(preint_.covariance.trace() / 9.0));
    const T scale = T(info_scale);

    Eigen::Map<Eigen::Matrix<T, 9, 1>> res(residual);
    res.template segment<3>(0) = r_p * scale;
    res.template segment<3>(3) = r_q * scale;
    res.template segment<3>(6) = r_v * scale;

    return true;
  }

  static ceres::CostFunction* Create(ImuPreintegration preint) {
    return new ceres::AutoDiffCostFunction<ImuPreintegrationCost, 9, 7, 3, 6, 7, 3, 6>(
        new ImuPreintegrationCost(std::move(preint)));
  }

  ImuPreintegration preint_;
};

// ── GNSS Position Cost ────────────────────────────────────────────────────────
//
// Residual (3-DOF): R^{1/2} * (p_enu - meas_enu)
// Parameter block: pose[7] = [px, py, pz, qx, qy, qz, qw]
//
struct GnssPositionCost {
  GnssPositionCost(Eigen::Vector3d meas_enu_m, Eigen::Matrix3d cov_m2)
      : meas_(std::move(meas_enu_m)) {
    // sqrt information matrix via LDLT
    const Eigen::LLT<Eigen::Matrix3d> llt(cov_m2.inverse());
    sqrt_info_ = llt.matrixL().transpose();
  }

  template <typename T>
  bool operator()(const T* pose, T* residual) const {
    Eigen::Matrix<T, 3, 1> p(pose[0], pose[1], pose[2]);
    Eigen::Matrix<T, 3, 1> r = sqrt_info_.template cast<T>() * (p - meas_.template cast<T>());
    residual[0] = r(0); residual[1] = r(1); residual[2] = r(2);
    return true;
  }

  static ceres::CostFunction* Create(Eigen::Vector3d meas, Eigen::Matrix3d cov) {
    return new ceres::AutoDiffCostFunction<GnssPositionCost, 3, 7>(
        new GnssPositionCost(std::move(meas), std::move(cov)));
  }

  Eigen::Vector3d meas_;
  Eigen::Matrix3d sqrt_info_;
};

// ── GNSS Velocity Cost ────────────────────────────────────────────────────────
//
// Residual (3-DOF): R^{1/2} * (v_enu - meas_enu)
// Parameter block: vel[3]
//
struct GnssVelocityCost {
  GnssVelocityCost(Eigen::Vector3d meas_enu_mps, Eigen::Matrix3d cov_m2ps2)
      : meas_(std::move(meas_enu_mps)) {
    const Eigen::LLT<Eigen::Matrix3d> llt(cov_m2ps2.inverse());
    sqrt_info_ = llt.matrixL().transpose();
  }

  template <typename T>
  bool operator()(const T* vel, T* residual) const {
    Eigen::Matrix<T, 3, 1> v(vel[0], vel[1], vel[2]);
    Eigen::Matrix<T, 3, 1> r = sqrt_info_.template cast<T>() * (v - meas_.template cast<T>());
    residual[0] = r(0); residual[1] = r(1); residual[2] = r(2);
    return true;
  }

  static ceres::CostFunction* Create(Eigen::Vector3d meas, Eigen::Matrix3d cov) {
    return new ceres::AutoDiffCostFunction<GnssVelocityCost, 3, 3>(
        new GnssVelocityCost(std::move(meas), std::move(cov)));
  }

  Eigen::Vector3d meas_;
  Eigen::Matrix3d sqrt_info_;
};

// ── Bias Random Walk Cost (between consecutive keyframes) ─────────────────────
//
// Regularises IMU biases to follow a random walk.
// Residual (6-DOF): [bias_j - bias_i] / sigma
//
struct BiasRandomWalkCost {
  BiasRandomWalkCost(double accel_sigma, double gyro_sigma)
      : accel_sigma_(accel_sigma), gyro_sigma_(gyro_sigma) {}

  template <typename T>
  bool operator()(const T* bias_i, const T* bias_j, T* residual) const {
    const T inv_a = T(1.0 / accel_sigma_);
    const T inv_g = T(1.0 / gyro_sigma_);
    residual[0] = (bias_j[0] - bias_i[0]) * inv_a;
    residual[1] = (bias_j[1] - bias_i[1]) * inv_a;
    residual[2] = (bias_j[2] - bias_i[2]) * inv_a;
    residual[3] = (bias_j[3] - bias_i[3]) * inv_g;
    residual[4] = (bias_j[4] - bias_i[4]) * inv_g;
    residual[5] = (bias_j[5] - bias_i[5]) * inv_g;
    return true;
  }

  static ceres::CostFunction* Create(double accel_sigma, double gyro_sigma) {
    return new ceres::AutoDiffCostFunction<BiasRandomWalkCost, 6, 6, 6>(
        new BiasRandomWalkCost(accel_sigma, gyro_sigma));
  }

  double accel_sigma_;
  double gyro_sigma_;
};

}  // namespace falconguide::estimation::ceres_backend

#endif  // FALCONGUIDE_HAVE_CERES
