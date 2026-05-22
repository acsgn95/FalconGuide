#include "falconguide/estimation/backends/ekf/measurement_models/dvl.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/ekf_update.hpp"

#include "falconguide/core/math.hpp"

#include <variant>

namespace falconguide::estimation::ekf {

Dvl::Dvl(DvlOptions options) : options_(std::move(options)) {}

bool Dvl::CanHandle(const SensorMeasurement& measurement) const {
    return std::holds_alternative<core::DvlMeasurement>(measurement);
}

EstimatorUpdateResult Dvl::Apply(NominalState& nominal, Eigen::VectorXd& error_state, Eigen::MatrixXd& covariance,
                                 const StateLayout& layout, const SensorMeasurement& measurement,
                                 const UpdateContext& /*context*/) {
    const auto& dvl = std::get<core::DvlMeasurement>(measurement);
    if (dvl.validity != core::MeasurementValidity::Valid) return EstimatorUpdateResult::Rejected;
    if (options_.bottom_track_only && dvl.reference != core::DvlReference::BottomTrack)
        return EstimatorUpdateResult::Rejected;

    const int n = layout.TotalSize();
    const auto& vel_seg = layout.Get(StateSegmentId::Velocity);
    const auto& att_seg = layout.Get(StateSegmentId::Attitude);

    const Eigen::Matrix3d R = nominal.orientation_body_to_enu.toRotationMatrix();

    // A = R_dvl_to_body^T * R_body_to_enu^T
    const Eigen::Matrix3d A = options_.dvl_to_body_rotation.transpose() * R.transpose();
    const Eigen::Vector3d v_pred = A * nominal.velocity_enu_mps.eigen();
    const Eigen::Vector3d v_obs = dvl.velocity_mps.eigen();

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, n);
    H.block<3, 3>(0, vel_seg.offset) = A;
    // dh/dδθ = R_dvl^T * SkewSymmetric(R^T * v_enu)
    H.block<3, 3>(0, att_seg.offset) = options_.dvl_to_body_rotation.transpose() *
                                       core::SkewSymmetric(R.transpose() * nominal.velocity_enu_mps.eigen());

    const Eigen::VectorXd dz = v_obs - v_pred;

    const double s = options_.sigma_mps.value_or(0.05);
    Eigen::MatrixXd R_meas;
    if (dvl.velocity_covariance_m2ps2.trace() > 1e-12) {
        R_meas = dvl.velocity_covariance_m2ps2;
    } else {
        R_meas = Eigen::Matrix3d::Identity() * (s * s);
    }

    return EkfUpdate(error_state, covariance, H, dz, R_meas, options_.innovation_gate);
}

}  // namespace falconguide::estimation::ekf
