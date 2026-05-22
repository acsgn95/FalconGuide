#include "falconguide/estimation/backends/ekf/measurement_models/aiding_solution.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/ekf_update.hpp"

#include "falconguide/core/math.hpp"

#include <variant>

namespace falconguide::estimation::ekf {

AidingSolution::AidingSolution(AidingSolutionOptions options) : options_(std::move(options)) {}

bool AidingSolution::CanHandle(const SensorMeasurement& measurement) const {
    return std::holds_alternative<core::AidingSolution>(measurement);
}

EstimatorUpdateResult AidingSolution::Apply(NominalState& nominal, Eigen::VectorXd& error_state,
                                            Eigen::MatrixXd& covariance, const StateLayout& layout,
                                            const SensorMeasurement& measurement, const UpdateContext& context) {
    const auto& aid = std::get<core::AidingSolution>(measurement);
    if (aid.validity != core::MeasurementValidity::Valid) return EstimatorUpdateResult::Rejected;
    if (aid.match_score && *aid.match_score < options_.min_match_score) return EstimatorUpdateResult::Rejected;
    if (!context.ltp) return EstimatorUpdateResult::NotInitialized;

    const int n = layout.TotalSize();
    const auto& pos_seg = layout.Get(StateSegmentId::Position);
    const auto& vel_seg = layout.Get(StateSegmentId::Velocity);
    const auto& att_seg = layout.Get(StateSegmentId::Attitude);

    const Eigen::Matrix3d R_ecef_to_enu = context.ltp->ecef_to_enu_rotation();

    // ── Collect active sub-observations ──────────────────────────────────────
    // We apply each sub-block independently to keep code clear and allow partial failures.

    auto result = EstimatorUpdateResult::Rejected;

    // ── Position update ───────────────────────────────────────────────────────
    if (aid.position_ecef_m) {
        const Eigen::Vector3d p_enu = context.ltp->EcefToEnu(*aid.position_ecef_m).eigen();
        const Eigen::Vector3d dz_p = p_enu - nominal.position_enu_m.eigen();

        Eigen::Matrix3d R_pos;
        if (options_.position_sigma_m) {
            const double s = *options_.position_sigma_m;
            R_pos = Eigen::Matrix3d::Identity() * (s * s);
        } else {
            R_pos = R_ecef_to_enu * aid.position_covariance_ecef_m2 * R_ecef_to_enu.transpose();
            if (R_pos.trace() < 1e-12) {
                const double s = 10.0;
                R_pos = Eigen::Matrix3d::Identity() * (s * s);
            }
        }

        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, n);
        H.block<3, 3>(0, pos_seg.offset) = Eigen::Matrix3d::Identity();
        if (EkfUpdate(error_state, covariance, H, dz_p, R_pos, options_.innovation_gate) ==
            EstimatorUpdateResult::Accepted) {
            result = EstimatorUpdateResult::Accepted;
        }
    }

    // ── Velocity update ───────────────────────────────────────────────────────
    if (aid.velocity_ecef_mps) {
        const Eigen::Vector3d v_enu = R_ecef_to_enu * aid.velocity_ecef_mps->eigen();
        const Eigen::Vector3d dz_v = v_enu - nominal.velocity_enu_mps.eigen();

        Eigen::Matrix3d R_vel;
        if (options_.velocity_sigma_mps) {
            const double s = *options_.velocity_sigma_mps;
            R_vel = Eigen::Matrix3d::Identity() * (s * s);
        } else {
            R_vel = R_ecef_to_enu * aid.velocity_covariance_ecef_mps2 * R_ecef_to_enu.transpose();
            if (R_vel.trace() < 1e-12) {
                const double s = 1.0;
                R_vel = Eigen::Matrix3d::Identity() * (s * s);
            }
        }

        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, n);
        H.block<3, 3>(0, vel_seg.offset) = Eigen::Matrix3d::Identity();
        if (EkfUpdate(error_state, covariance, H, dz_v, R_vel, options_.innovation_gate) ==
            EstimatorUpdateResult::Accepted) {
            result = EstimatorUpdateResult::Accepted;
        }
    }

    // ── Attitude update (selected axes) ───────────────────────────────────────
    if (aid.orientation_body_to_ecef && aid.orientation_validity.any()) {
        const Eigen::Quaterniond q_body_to_ecef = *aid.orientation_body_to_ecef;
        // Convert body→ECEF to body→ENU
        const Eigen::Quaterniond q_ecef_to_enu(R_ecef_to_enu);
        const Eigen::Quaterniond q_obs = (q_ecef_to_enu * q_body_to_ecef).normalized();
        const Eigen::Vector3d d_theta = core::LogMapSo3(nominal.orientation_body_to_enu.conjugate() * q_obs);

        const double s = options_.attitude_sigma_rad.value_or(0.05);
        const Eigen::Matrix3d R_att = Eigen::Matrix3d::Identity() * (s * s);

        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, n);
        H.block<3, 3>(0, att_seg.offset) = Eigen::Matrix3d::Identity();

        // Zero out rows for axes that the aiding source doesn't observe
        if (!aid.orientation_validity.roll) H.row(0).setZero();
        if (!aid.orientation_validity.pitch) H.row(1).setZero();
        if (!aid.orientation_validity.yaw) H.row(2).setZero();

        Eigen::VectorXd dz_a(3);
        dz_a(0) = aid.orientation_validity.roll ? d_theta.x() : 0.0;
        dz_a(1) = aid.orientation_validity.pitch ? d_theta.y() : 0.0;
        dz_a(2) = aid.orientation_validity.yaw ? d_theta.z() : 0.0;

        if (EkfUpdate(error_state, covariance, H, dz_a, R_att, options_.innovation_gate) ==
            EstimatorUpdateResult::Accepted) {
            result = EstimatorUpdateResult::Accepted;
        }
    }

    return result;
}

}  // namespace falconguide::estimation::ekf
