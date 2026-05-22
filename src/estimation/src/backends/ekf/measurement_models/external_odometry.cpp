#include "falconguide/estimation/backends/ekf/measurement_models/external_odometry.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/ekf_update.hpp"

#include "falconguide/core/math.hpp"

#include <variant>

namespace falconguide::estimation::ekf {

ExternalOdometry::ExternalOdometry(ExternalOdometryOptions options) : options_(std::move(options)) {}

bool ExternalOdometry::CanHandle(const SensorMeasurement& measurement) const {
    return std::holds_alternative<core::ExternalOdometryMeasurement>(measurement);
}

EstimatorUpdateResult ExternalOdometry::Apply(NominalState& nominal, Eigen::VectorXd& error_state,
                                              Eigen::MatrixXd& covariance, const StateLayout& layout,
                                              const SensorMeasurement& measurement, const UpdateContext& context) {
    const auto& eo = std::get<core::ExternalOdometryMeasurement>(measurement);
    if (eo.validity != core::MeasurementValidity::Valid) return EstimatorUpdateResult::Rejected;
    if (!context.ltp) return EstimatorUpdateResult::NotInitialized;

    const int n = layout.TotalSize();
    const auto& pos_seg = layout.Get(StateSegmentId::Position);
    const auto& vel_seg = layout.Get(StateSegmentId::Velocity);
    const auto& att_seg = layout.Get(StateSegmentId::Attitude);
    const Eigen::Matrix3d R_ecef_to_enu = context.ltp->ecef_to_enu_rotation();

    auto result = EstimatorUpdateResult::Rejected;

    // ── Position ──────────────────────────────────────────────────────────────
    if (options_.use_position) {
        const Eigen::Vector3d p_enu = context.ltp->EcefToEnu(eo.position_ecef_m).eigen();
        const Eigen::Vector3d dz_p = p_enu - nominal.position_enu_m.eigen();

        const double s = options_.position_sigma_m.value_or(1.0);
        const Eigen::Matrix3d R_pos = Eigen::Matrix3d::Identity() * (s * s);

        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, n);
        H.block<3, 3>(0, pos_seg.offset) = Eigen::Matrix3d::Identity();
        if (EkfUpdate(error_state, covariance, H, dz_p, R_pos, options_.innovation_gate) ==
            EstimatorUpdateResult::Accepted) {
            result = EstimatorUpdateResult::Accepted;
        }
    }

    // ── Velocity ──────────────────────────────────────────────────────────────
    if (options_.use_velocity) {
        const Eigen::Vector3d v_enu = R_ecef_to_enu * eo.velocity_ecef_mps.eigen();
        const Eigen::Vector3d dz_v = v_enu - nominal.velocity_enu_mps.eigen();

        const double s = options_.velocity_sigma_mps.value_or(0.1);
        const Eigen::Matrix3d R_vel = Eigen::Matrix3d::Identity() * (s * s);

        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, n);
        H.block<3, 3>(0, vel_seg.offset) = Eigen::Matrix3d::Identity();
        if (EkfUpdate(error_state, covariance, H, dz_v, R_vel, options_.innovation_gate) ==
            EstimatorUpdateResult::Accepted) {
            result = EstimatorUpdateResult::Accepted;
        }
    }

    // ── Attitude ──────────────────────────────────────────────────────────────
    if (options_.use_orientation) {
        const Eigen::Quaterniond q_ecef_to_enu(R_ecef_to_enu);
        const Eigen::Quaterniond q_obs = (q_ecef_to_enu * eo.orientation_body_to_ecef).normalized();
        const Eigen::Vector3d d_theta = core::LogMapSo3(nominal.orientation_body_to_enu.conjugate() * q_obs);

        const double s = options_.orientation_sigma_rad.value_or(0.02);
        const Eigen::Matrix3d R_att = Eigen::Matrix3d::Identity() * (s * s);

        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, n);
        H.block<3, 3>(0, att_seg.offset) = Eigen::Matrix3d::Identity();
        if (EkfUpdate(error_state, covariance, H, d_theta, R_att, options_.innovation_gate) ==
            EstimatorUpdateResult::Accepted) {
            result = EstimatorUpdateResult::Accepted;
        }
    }

    return result;
}

}  // namespace falconguide::estimation::ekf
