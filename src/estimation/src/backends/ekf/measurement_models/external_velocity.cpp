#include "falconguide/estimation/backends/ekf/measurement_models/external_velocity.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/ekf_update.hpp"

#include "falconguide/core/math.hpp"

#include <variant>

namespace falconguide::estimation::ekf {

ExternalVelocity::ExternalVelocity(ExternalVelocityOptions options) : options_(std::move(options)) {}

bool ExternalVelocity::CanHandle(const SensorMeasurement& measurement) const {
  return std::holds_alternative<core::ExternalVelocityMeasurement>(measurement);
}

EstimatorUpdateResult ExternalVelocity::Apply(
    NominalState& nominal,
    Eigen::VectorXd& error_state,
    Eigen::MatrixXd& covariance,
    const StateLayout& layout,
    const SensorMeasurement& measurement,
    const UpdateContext& /*context*/) {

  const auto& ev = std::get<core::ExternalVelocityMeasurement>(measurement);
  if (ev.validity != core::MeasurementValidity::Valid) return EstimatorUpdateResult::Rejected;

  const int n = layout.TotalSize();
  const auto& vel_seg = layout.Get(StateSegmentId::Velocity);
  const auto& att_seg = layout.Get(StateSegmentId::Attitude);

  const Eigen::Matrix3d R      = nominal.orientation_body_to_enu.toRotationMatrix();
  const Eigen::Vector3d v_body = R.transpose() * nominal.velocity_enu_mps.eigen();
  const Eigen::Vector3d v_obs  = ev.linear_velocity_body_mps.eigen();

  const int m_lin = options_.use_angular_rate ? 6 : 3;
  Eigen::MatrixXd H  = Eigen::MatrixXd::Zero(m_lin, n);
  Eigen::VectorXd dz = Eigen::VectorXd::Zero(m_lin);

  H.block<3, 3>(0, vel_seg.offset) = R.transpose();
  H.block<3, 3>(0, att_seg.offset) = core::SkewSymmetric(v_body);
  dz.head<3>()                      = v_obs - v_body;

  if (options_.use_angular_rate && layout.Has(StateSegmentId::GyroBias)) {
    const auto& bg_seg = layout.Get(StateSegmentId::GyroBias);
    const Eigen::Vector3d omega_pred = options_.latest_gyro_radps - nominal.gyro_bias_radps.eigen();
    const Eigen::Vector3d omega_obs  = ev.angular_rate_body_radps.eigen();
    H.block<3, 3>(3, bg_seg.offset) = -Eigen::Matrix3d::Identity();
    dz.tail<3>()                     = omega_obs - omega_pred;
  }

  const double s_lin = options_.sigma_linear_mps.value_or(0.1);
  const double s_ang = options_.sigma_angular_radps.value_or(0.05);

  Eigen::MatrixXd R_meas = Eigen::MatrixXd::Zero(m_lin, m_lin);
  R_meas.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (s_lin * s_lin);
  if (options_.use_angular_rate) {
    R_meas.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (s_ang * s_ang);
  }

  return EkfUpdate(error_state, covariance, H, dz, R_meas, options_.innovation_gate);
}

}  // namespace falconguide::estimation::ekf
