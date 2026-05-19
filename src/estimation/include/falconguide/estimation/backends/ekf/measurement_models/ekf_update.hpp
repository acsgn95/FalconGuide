#pragma once

#include "falconguide/core/math.hpp"
#include "falconguide/estimation/estimator_interface.hpp"

#include <Eigen/Core>

namespace falconguide::estimation::ekf {

// Full Kalman update step with Joseph-form covariance and optional Mahalanobis gate.
// H: m×n   dz: m×1   R_meas: m×m   mahal_gate: 0 = disabled
inline EstimatorUpdateResult EkfUpdate(
    Eigen::VectorXd& error_state,
    Eigen::MatrixXd& covariance,
    const Eigen::MatrixXd& H,
    const Eigen::VectorXd& dz,
    const Eigen::MatrixXd& R_meas,
    double mahal_gate = 0.0) {

  const int n = static_cast<int>(covariance.rows());
  const Eigen::MatrixXd PHt = covariance * H.transpose();
  const Eigen::MatrixXd S   = H * PHt + R_meas;

  if (mahal_gate > 0.0) {
    const double mahal = dz.transpose() * S.ldlt().solve(dz);
    if (mahal > mahal_gate) return EstimatorUpdateResult::Rejected;
  }

  const Eigen::MatrixXd K   = PHt * S.inverse();
  error_state += K * dz;

  const Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(n, n) - K * H;
  covariance = IKH * covariance * IKH.transpose() + K * R_meas * K.transpose();
  covariance = core::SymmetrizeCovariance(covariance);
  return EstimatorUpdateResult::Accepted;
}

}  // namespace falconguide::estimation::ekf
