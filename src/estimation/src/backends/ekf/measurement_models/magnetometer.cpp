#include "falconguide/estimation/backends/ekf/measurement_models/magnetometer.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/ekf_update.hpp"

#include "falconguide/core/math.hpp"

#include <variant>

namespace falconguide::estimation::ekf {

Magnetometer::Magnetometer(MagnetometerOptions options) : options_(std::move(options)) {}

bool Magnetometer::CanHandle(const SensorMeasurement& measurement) const {
    return std::holds_alternative<core::MagnetometerMeasurement>(measurement);
}

EstimatorUpdateResult Magnetometer::Apply(NominalState& nominal, Eigen::VectorXd& error_state,
                                          Eigen::MatrixXd& covariance, const StateLayout& layout,
                                          const SensorMeasurement& measurement, const UpdateContext& /*context*/) {
    const auto& mag = std::get<core::MagnetometerMeasurement>(measurement);
    if (mag.validity != core::MeasurementValidity::Valid) return EstimatorUpdateResult::Rejected;

    const int n = layout.TotalSize();
    const auto& att_seg = layout.Get(StateSegmentId::Attitude);

    // ── Predicted measurement: body-frame field from reference ────────────────
    const Eigen::Matrix3d R = nominal.orientation_body_to_enu.toRotationMatrix();
    const Eigen::Vector3d b_pred = R.transpose() * options_.reference_field_enu_tesla;

    // ── Observation ───────────────────────────────────────────────────────────
    const Eigen::Vector3d b_obs = mag.magnetic_field_tesla.eigen();
    const Eigen::VectorXd dz = b_obs - b_pred;

    // ── H matrix (3×n): attitude block only ──────────────────────────────────
    // dh/dδθ = SkewSymmetric(b_pred)  (right-body perturbation derivation)
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, n);
    H.block<3, 3>(0, att_seg.offset) = core::SkewSymmetric(b_pred);

    // ── Noise covariance (3×3) ────────────────────────────────────────────────
    const double s = options_.sigma_tesla.value_or(options_.reference_field_enu_tesla.norm() * 0.05);
    const Eigen::MatrixXd R_meas = Eigen::Matrix3d::Identity() * (s * s);

    return EkfUpdate(error_state, covariance, H, dz, R_meas, options_.innovation_gate);
}

}  // namespace falconguide::estimation::ekf
