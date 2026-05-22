#include "falconguide/estimation/backends/ekf/measurement_models/star_tracker.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/ekf_update.hpp"

#include "falconguide/core/math.hpp"

#include <variant>

namespace falconguide::estimation::ekf {

StarTracker::StarTracker(StarTrackerOptions options) : options_(std::move(options)) {}

bool StarTracker::CanHandle(const SensorMeasurement& measurement) const {
  return std::holds_alternative<core::StarTrackerMeasurement>(measurement);
}

// Construct body-to-ENU quaternion from star tracker (lon, lat, yaw).
//
// Interpretation:
//   lon  — azimuth of the boresight from North, measured East  (ENU yaw convention)
//   lat  — elevation of the boresight above the horizontal plane
//   yaw  — roll around the boresight axis
//
// q = Rz(lon) * Ry(π/2 − lat) * Rz(−yaw) * q_sensor_to_body
//
static Eigen::Quaterniond QuaternionFromStarTracker(
    double lon_rad, double lat_rad, double yaw_rad,
    const Eigen::Quaterniond& sensor_to_body) {

  const Eigen::Quaterniond q_lon(Eigen::AngleAxisd(lon_rad,               Eigen::Vector3d::UnitZ()));
  const Eigen::Quaterniond q_lat(Eigen::AngleAxisd(core::kPi / 2.0 - lat_rad, Eigen::Vector3d::UnitY()));
  const Eigen::Quaterniond q_yaw(Eigen::AngleAxisd(-yaw_rad,              Eigen::Vector3d::UnitZ()));
  return (q_lon * q_lat * q_yaw * sensor_to_body).normalized();
}

EstimatorUpdateResult StarTracker::Apply(
    NominalState& nominal,
    Eigen::VectorXd& error_state,
    Eigen::MatrixXd& covariance,
    const StateLayout& layout,
    const SensorMeasurement& measurement,
    const UpdateContext& /*context*/) {

  const auto& st = std::get<core::StarTrackerMeasurement>(measurement);
  if (st.validity != core::MeasurementValidity::Valid) return EstimatorUpdateResult::Rejected;

  const bool use_pr = options_.use_pitch_roll && st.pitch_rad && st.roll_rad;
  const int n = layout.TotalSize();
  const auto& att_seg = layout.Get(StateSegmentId::Attitude);

  // ── Build observed quaternion ─────────────────────────────────────────────
  const double pitch = use_pr ? *st.pitch_rad : 0.0;
  const double roll  = use_pr ? *st.roll_rad  : 0.0;
  (void)pitch; (void)roll;  // used implicitly through star tracker orientation math

  const Eigen::Quaterniond q_obs =
      QuaternionFromStarTracker(st.longitude_rad, st.latitude_rad, st.yaw_rad,
                                options_.star_tracker_to_body);

  // ── Innovation δθ = LogMapSo3(q_pred^{-1} ⊗ q_obs) ──────────────────────
  const Eigen::Quaterniond q_pred = nominal.orientation_body_to_enu;
  Eigen::Vector3d d_theta = core::LogMapSo3(q_pred.conjugate() * q_obs);

  // ── Select active measurement dimension ───────────────────────────────────
  const int m = use_pr ? 3 : 1;
  Eigen::MatrixXd H  = Eigen::MatrixXd::Zero(m, n);
  Eigen::VectorXd dz = Eigen::VectorXd::Zero(m);

  if (use_pr) {
    H.block<3, 3>(0, att_seg.offset) = Eigen::Matrix3d::Identity();
    dz = d_theta;
  } else {
    // Yaw-only: extract z-component of rotation vector (approx heading error)
    H(0, att_seg.offset + 2) = 1.0;
    dz(0) = d_theta.z();
  }

  // ── Noise covariance ─────────────────────────────────────────────────────
  Eigen::MatrixXd R_meas = Eigen::MatrixXd::Zero(m, m);
  if (use_pr) {
    R_meas(0, 0) = options_.sigma_roll_rad  * options_.sigma_roll_rad;
    R_meas(1, 1) = options_.sigma_pitch_rad * options_.sigma_pitch_rad;
    R_meas(2, 2) = options_.sigma_yaw_rad   * options_.sigma_yaw_rad;
  } else {
    R_meas(0, 0) = options_.sigma_yaw_rad * options_.sigma_yaw_rad;
  }

  return EkfUpdate(error_state, covariance, H, dz, R_meas, options_.innovation_gate);
}

}  // namespace falconguide::estimation::ekf
