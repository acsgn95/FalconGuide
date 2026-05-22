#include "falconguide/estimation/backends/ekf/measurement_models/airspeed.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/ekf_update.hpp"

#include "falconguide/core/math.hpp"

#include <variant>

namespace falconguide::estimation::ekf {

Airspeed::Airspeed(AirspeedOptions options) : options_(std::move(options)) {}

bool Airspeed::CanHandle(const SensorMeasurement& measurement) const {
    return std::holds_alternative<core::AirspeedMeasurement>(measurement);
}

EstimatorUpdateResult Airspeed::Apply(NominalState& nominal, Eigen::VectorXd& error_state, Eigen::MatrixXd& covariance,
                                      const StateLayout& layout, const SensorMeasurement& measurement,
                                      const UpdateContext& /*context*/) {
    const auto& asp = std::get<core::AirspeedMeasurement>(measurement);
    if (asp.validity != core::MeasurementValidity::Valid) return EstimatorUpdateResult::Rejected;
    if (asp.airspeed_mps < 0.0) return EstimatorUpdateResult::Rejected;

    const int n = layout.TotalSize();
    const auto& vel_seg = layout.Get(StateSegmentId::Velocity);
    const auto& att_seg = layout.Get(StateSegmentId::Attitude);

    // ── Predicted airspeed ────────────────────────────────────────────────────
    const Eigen::Matrix3d R = nominal.orientation_body_to_enu.toRotationMatrix();
    const Eigen::Vector3d v_air = nominal.velocity_enu_mps.eigen() - options_.wind_enu_mps;
    const Eigen::Vector3d v_body = R.transpose() * v_air;
    const double h_pred = v_body.norm();

    if (h_pred < 1e-3) return EstimatorUpdateResult::Rejected;  // near-zero velocity, ill-conditioned

    const Eigen::Vector3d u_body = v_body / h_pred;  // unit vector

    // ── H matrix (1×n) ───────────────────────────────────────────────────────
    // H_vel = u_body^T * R^T
    // H_att = u_body^T * SkewSymmetric(v_body)
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, n);
    H.block<1, 3>(0, vel_seg.offset) = (u_body.transpose() * R.transpose()).eval();
    H.block<1, 3>(0, att_seg.offset) = (u_body.transpose() * core::SkewSymmetric(v_body)).eval();

    const Eigen::VectorXd dz = Eigen::VectorXd::Constant(1, asp.airspeed_mps - h_pred);
    const Eigen::MatrixXd R_meas = Eigen::MatrixXd::Constant(1, 1, options_.sigma_mps * options_.sigma_mps);

    return EkfUpdate(error_state, covariance, H, dz, R_meas, options_.innovation_gate);
}

}  // namespace falconguide::estimation::ekf
