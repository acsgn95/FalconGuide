#pragma once

/**
 * @file imu_preintegrator.hpp
 * @brief Inline IMU preintegration implementation for Ceres factors.
 */

#include "falconguide/core/sensors/imu.hpp"
#include "falconguide/estimation/backends/ceres/ceres_state.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace falconguide::estimation::ceres_backend {

// ── ImuPreintegrator
// ──────────────────────────────────────────────────────────
//
// Implements Forster et al. manifold IMU preintegration (TRO 2017, simplified).
// Integrates raw IMU from one keyframe to the next, accumulating:
//   Δp — position increment in the local frame of keyframe_i
//   Δq — rotation increment (body_i → body_j)
//   Δv — velocity increment in the local frame of keyframe_i
//
// Midpoint integration is used for both the delta states and the 9×9
// covariance propagation.  First-order bias Jacobians are computed to allow
// the Ceres residual to apply a linearised correction when biases change,
// avoiding full re-integration on every solver iteration.
//
class ImuPreintegrator {
public:
  /// @brief Noise parameters used during preintegration.
  struct Params {
    double accel_noise_density_mps2_sqrthz{
        3.0e-3}; ///< Accelerometer white-noise density.
    double gyro_noise_density_radps_sqrthz{
        1.5e-4}; ///< Gyroscope white-noise density.
    double accel_random_walk_mps3_sqrthz{
        3.0e-5}; ///< Accelerometer bias random walk.
    double gyro_random_walk_radps2_sqrthz{
        2.0e-6}; ///< Gyroscope bias random walk.
  };

  /// @brief Constructs an empty preintegrator.
  ImuPreintegrator() = default;

  /// @brief Resets to a new integration window.
  void Reset(Eigen::Vector3d accel_bias_mps2, Eigen::Vector3d gyro_bias_radps,
             const core::Timestamp &t0, Params params = {});

  /// @brief Integrates one IMU sample.
  /// @note Samples must arrive in chronological order.
  void Integrate(const core::ImuMeasurement &imu);

  /// @brief Finalizes and returns the preintegration result.
  ImuPreintegration Finalize(const core::Timestamp &t1) const;

  /// @brief Returns the preintegrated position delta.
  [[nodiscard]] const Eigen::Vector3d &DeltaP() const { return delta_p_; }
  /// @brief Returns the preintegrated rotation delta.
  [[nodiscard]] const Eigen::Quaterniond &DeltaQ() const { return delta_q_; }
  /// @brief Returns the preintegrated velocity delta.
  [[nodiscard]] const Eigen::Vector3d &DeltaV() const { return delta_v_; }
  /// @brief Returns total integrated time in seconds.
  [[nodiscard]] double DeltaT() const { return delta_t_; }
  /// @brief Returns the number of integrated samples.
  [[nodiscard]] std::size_t NumSamples() const { return num_samples_; }

private:
  // Quaternion exponential map:  φ → q = [cos(|φ|/2), sin(|φ|/2)*φ/|φ|]
  static Eigen::Quaterniond ExpMap(const Eigen::Vector3d &phi);

  // Skew-symmetric matrix of v
  static Eigen::Matrix3d Skew(const Eigen::Vector3d &v);

  Params params_;
  core::Timestamp t0_;

  // Bias linearisation point
  Eigen::Vector3d ba_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d bg_{Eigen::Vector3d::Zero()};

  // Delta state (preintegrated quantities)
  Eigen::Vector3d delta_p_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond delta_q_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d delta_v_{Eigen::Vector3d::Zero()};

  // Covariance [dp(3), dθ(3), dv(3)] × [dp(3), dθ(3), dv(3)]
  Eigen::Matrix<double, 9, 9> cov_{Eigen::Matrix<double, 9, 9>::Zero()};

  // Jacobians of delta state w.r.t. biases (for first-order correction)
  Eigen::Matrix<double, 3, 3> J_dp_dba_{Eigen::Matrix<double, 3, 3>::Zero()};
  Eigen::Matrix<double, 3, 3> J_dp_dbg_{Eigen::Matrix<double, 3, 3>::Zero()};
  Eigen::Matrix<double, 3, 3> J_dq_dbg_{Eigen::Matrix<double, 3, 3>::Zero()};
  Eigen::Matrix<double, 3, 3> J_dv_dba_{Eigen::Matrix<double, 3, 3>::Zero()};
  Eigen::Matrix<double, 3, 3> J_dv_dbg_{Eigen::Matrix<double, 3, 3>::Zero()};

  double delta_t_{0.0};
  bool has_prev_{false};
  core::Timestamp prev_ts_;
  core::ImuMeasurement prev_imu_;
  std::size_t num_samples_{0};
};

// ── ImuPreintegrator inline implementation
// ────────────────────────────────────

inline Eigen::Quaterniond ImuPreintegrator::ExpMap(const Eigen::Vector3d &phi) {
  const double angle = phi.norm();
  if (angle < 1e-10) {
    return {1.0, 0.5 * phi.x(), 0.5 * phi.y(), 0.5 * phi.z()};
  }
  const double s = std::sin(0.5 * angle) / angle;
  return {std::cos(0.5 * angle), s * phi.x(), s * phi.y(), s * phi.z()};
}

inline Eigen::Matrix3d ImuPreintegrator::Skew(const Eigen::Vector3d &v) {
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

inline void ImuPreintegrator::Reset(Eigen::Vector3d accel_bias,
                                    Eigen::Vector3d gyro_bias,
                                    const core::Timestamp &t0, Params params) {
  params_ = params;
  t0_ = t0;
  ba_ = std::move(accel_bias);
  bg_ = std::move(gyro_bias);
  delta_p_.setZero();
  delta_q_.setIdentity();
  delta_v_.setZero();
  cov_.setZero();
  J_dp_dba_.setZero();
  J_dp_dbg_.setZero();
  J_dq_dbg_.setZero();
  J_dv_dba_.setZero();
  J_dv_dbg_.setZero();
  delta_t_ = 0.0;
  has_prev_ = false;
  num_samples_ = 0;
}

inline void ImuPreintegrator::Integrate(const core::ImuMeasurement &imu) {
  if (!has_prev_) {
    prev_ts_ = imu.timestamp;
    prev_imu_ = imu;
    has_prev_ = true;
    return;
  }

  const double dt = (imu.timestamp.steady - prev_ts_.steady).seconds();
  if (dt <= 0.0 || dt > 1.0) {
    prev_ts_ = imu.timestamp;
    prev_imu_ = imu;
    return;
  }

  // Midpoint accelerometer and gyroscope (bias-corrected)
  const Eigen::Vector3d a0 = prev_imu_.specific_force_mps2.eigen() - ba_;
  const Eigen::Vector3d a1 = imu.specific_force_mps2.eigen() - ba_;
  const Eigen::Vector3d w0 = prev_imu_.angular_rate_radps.eigen() - bg_;
  const Eigen::Vector3d w1 = imu.angular_rate_radps.eigen() - bg_;

  const Eigen::Vector3d a_mid = 0.5 * (a0 + a1);
  const Eigen::Vector3d w_mid = 0.5 * (w0 + w1);

  // Rotate acceleration to delta_q frame (world-relative)
  const Eigen::Matrix3d R = delta_q_.toRotationMatrix();
  const Eigen::Vector3d a_world = R * a_mid;

  // Integrate delta state (position, velocity, rotation)
  delta_p_ += delta_v_ * dt + 0.5 * a_world * dt * dt;
  delta_v_ += a_world * dt;
  delta_q_ = (delta_q_ * ExpMap(w_mid * dt)).normalized();

  // ── Covariance propagation ─────────────────────────────────────────────────
  // F blocks [9×9], ordered as [dp, dθ, dv]
  // Continuous-time discrete approximation: P_k+1 = F*P_k*F^T + G*Q*G^T
  const Eigen::Matrix3d Ra = R;
  const Eigen::Matrix3d Sa = Skew(a_mid);

  Eigen::Matrix<double, 9, 9> F = Eigen::Matrix<double, 9, 9>::Identity();
  F.block<3, 3>(0, 3) = -Ra * Sa * dt;                    // dp / dθ
  F.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * dt; // dp / dv
  F.block<3, 3>(3, 3) =
      Eigen::Matrix3d::Identity(); // dθ / dθ (eye − Skew*dt below)
  F.block<3, 3>(3, 3) -= Skew(w_mid) * dt;
  F.block<3, 3>(6, 3) = -Ra * Sa * dt; // dv / dθ

  // Noise input matrix G [9×6], noise = [accel(3), gyro(3)]
  Eigen::Matrix<double, 9, 6> G = Eigen::Matrix<double, 9, 6>::Zero();
  G.block<3, 3>(0, 0) = 0.5 * Ra * dt * dt;
  G.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * dt;
  G.block<3, 3>(6, 0) = Ra * dt;

  // Continuous noise spectral density → discrete
  const double qa = params_.accel_noise_density_mps2_sqrthz *
                    params_.accel_noise_density_mps2_sqrthz;
  const double qg = params_.gyro_noise_density_radps_sqrthz *
                    params_.gyro_noise_density_radps_sqrthz;
  Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
  Q.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * qa;
  Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * qg;

  cov_ = F * cov_ * F.transpose() + G * Q * G.transpose();

  // ── Bias Jacobians (first-order) ──────────────────────────────────────────
  // J_dv_dba, J_dp_dba — straightforward from preintegration formula
  J_dp_dba_ += J_dv_dba_ * dt + 0.5 * Ra * dt * dt;
  J_dp_dbg_ += J_dv_dbg_ * dt - 0.5 * Ra * Sa * J_dq_dbg_ * dt * dt;
  J_dv_dba_ += Ra * dt;
  J_dv_dbg_ -= Ra * Sa * J_dq_dbg_ * dt;
  J_dq_dbg_ = (Eigen::Matrix3d::Identity() - Skew(w_mid) * dt) * J_dq_dbg_ -
              Eigen::Matrix3d::Identity() * dt;

  delta_t_ += dt;
  prev_ts_ = imu.timestamp;
  prev_imu_ = imu;
  ++num_samples_;
}

inline ImuPreintegration
ImuPreintegrator::Finalize(const core::Timestamp &t1) const {
  ImuPreintegration p;
  p.start_timestamp = t0_;
  p.end_timestamp = t1;
  p.delta_p = delta_p_;
  p.delta_q = delta_q_;
  p.delta_v = delta_v_;
  p.covariance = cov_;

  // Pack bias Jacobians into the ImuPreintegration 9×3 blocks
  // Layout: [dp(3), dq(3), dv(3)] × bias(3)
  p.jacobian_dp_dba.block<3, 3>(0, 0) = J_dp_dba_;
  p.jacobian_dp_dba.block<3, 3>(3, 0) = J_dq_dbg_; // re-use field for dq/dbg
  p.jacobian_dp_dba.block<3, 3>(6, 0) = J_dv_dba_;
  p.jacobian_dp_dbg.block<3, 3>(0, 0) = J_dp_dbg_;
  p.jacobian_dp_dbg.block<3, 3>(6, 0) = J_dv_dbg_;

  p.linearisation_accel_bias_mps2 =
      core::Vec3<core::ImuFrame>(ba_.x(), ba_.y(), ba_.z());
  p.linearisation_gyro_bias_radps =
      core::Vec3<core::ImuFrame>(bg_.x(), bg_.y(), bg_.z());
  p.integration_time_s = delta_t_;
  p.num_samples = num_samples_;
  return p;
}

} // namespace falconguide::estimation::ceres_backend
