#pragma once

/**
 * @file ekf_update.hpp
 * @brief Joseph-form EKF update helper with optional Mahalanobis gating.
 */

#include "falconguide/core/math.hpp"
#include "falconguide/estimation/estimator_interface.hpp"

#include <Eigen/Core>

namespace falconguide::estimation::ekf {

/**
 * @brief Applies one linearized EKF measurement update.
 * @param error_state Error-state mean updated in place.
 * @param covariance Error-state covariance updated in place.
 * @param H Measurement Jacobian.
 * @param dz Innovation vector.
 * @param R_meas Measurement covariance.
 * @param mahal_gate Mahalanobis threshold; zero disables gating.
 * @return Accepted when the update is applied, otherwise Rejected.
 */
inline EstimatorUpdateResult
EkfUpdate(Eigen::VectorXd &error_state, Eigen::MatrixXd &covariance,
          const Eigen::MatrixXd &H, const Eigen::VectorXd &dz,
          const Eigen::MatrixXd &R_meas, double mahal_gate = 0.0) {

  const int n = static_cast<int>(covariance.rows());
  const Eigen::MatrixXd PHt = covariance * H.transpose();
  const Eigen::MatrixXd S = H * PHt + R_meas;

  if (mahal_gate > 0.0) {
    const double mahal = dz.transpose() * S.ldlt().solve(dz);
    if (mahal > mahal_gate)
      return EstimatorUpdateResult::Rejected;
  }

  const Eigen::MatrixXd K = PHt * S.inverse();
  error_state += K * dz;

  const Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(n, n) - K * H;
  covariance = IKH * covariance * IKH.transpose() + K * R_meas * K.transpose();
  covariance = core::SymmetrizeCovariance(covariance);
  return EstimatorUpdateResult::Accepted;
}

} // namespace falconguide::estimation::ekf
