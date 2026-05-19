#include "falconguide/estimation/backends/ekf/measurement_models/external_pose.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/ekf_update.hpp"

#include "falconguide/core/math.hpp"

#include <variant>

namespace falconguide::estimation::ekf {

ExternalPose::ExternalPose(ExternalPoseOptions options) : options_(std::move(options)) {}

bool ExternalPose::CanHandle(const SensorMeasurement& measurement) const {
  return std::holds_alternative<core::ExternalPoseMeasurement>(measurement);
}

EstimatorUpdateResult ExternalPose::Apply(
    NominalState& nominal,
    Eigen::VectorXd& error_state,
    Eigen::MatrixXd& covariance,
    const StateLayout& layout,
    const SensorMeasurement& measurement,
    const UpdateContext& context) {

  const auto& ep = std::get<core::ExternalPoseMeasurement>(measurement);
  if (ep.validity != core::MeasurementValidity::Valid) return EstimatorUpdateResult::Rejected;
  if (!context.ltp) return EstimatorUpdateResult::NotInitialized;

  const int n = layout.TotalSize();
  const auto& pos_seg = layout.Get(StateSegmentId::Position);
  const auto& att_seg = layout.Get(StateSegmentId::Attitude);
  const Eigen::Matrix3d R_ecef_to_enu = context.ltp->ecef_to_enu_rotation();

  auto result = EstimatorUpdateResult::Rejected;

  // ── Position ──────────────────────────────────────────────────────────────
  if (options_.use_position) {
    const Eigen::Vector3d p_enu = context.ltp->EcefToEnu(ep.position_ecef_m).eigen();
    const Eigen::Vector3d dz_p  = p_enu - nominal.position_enu_m.eigen();

    Eigen::Matrix3d R_pos;
    if (options_.position_sigma_m) {
      const double s = *options_.position_sigma_m;
      R_pos = Eigen::Matrix3d::Identity() * (s * s);
    } else {
      R_pos = R_ecef_to_enu * ep.covariance.block<3, 3>(0, 0) * R_ecef_to_enu.transpose();
      if (R_pos.trace() < 1e-12) R_pos = Eigen::Matrix3d::Identity() * 4.0;
    }

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, n);
    H.block<3, 3>(0, pos_seg.offset) = Eigen::Matrix3d::Identity();
    if (EkfUpdate(error_state, covariance, H, dz_p, R_pos, options_.innovation_gate)
        == EstimatorUpdateResult::Accepted) {
      result = EstimatorUpdateResult::Accepted;
    }
  }

  // ── Attitude ──────────────────────────────────────────────────────────────
  if (options_.use_orientation) {
    const Eigen::Quaterniond q_ecef_to_enu(R_ecef_to_enu);
    const Eigen::Quaterniond q_obs = (q_ecef_to_enu * ep.orientation_body_to_ecef).normalized();
    const Eigen::Vector3d d_theta  = core::LogMapSo3(nominal.orientation_body_to_enu.conjugate() * q_obs);

    const double s = options_.orientation_sigma_rad.value_or(0.02);
    const Eigen::Matrix3d R_att = Eigen::Matrix3d::Identity() * (s * s);

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, n);
    H.block<3, 3>(0, att_seg.offset) = Eigen::Matrix3d::Identity();
    if (EkfUpdate(error_state, covariance, H, d_theta, R_att, options_.innovation_gate)
        == EstimatorUpdateResult::Accepted) {
      result = EstimatorUpdateResult::Accepted;
    }
  }

  return result;
}

}  // namespace falconguide::estimation::ekf
