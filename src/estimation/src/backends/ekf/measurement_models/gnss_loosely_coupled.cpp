#include "falconguide/estimation/backends/ekf/measurement_models/gnss_loosely_coupled.hpp"

#include "falconguide/core/math.hpp"

#include <variant>

namespace falconguide::estimation::ekf {

GnssLooselyCoupled::GnssLooselyCoupled(GnssLooselyCoupledOptions options) : options_(std::move(options)) {}

bool GnssLooselyCoupled::CanHandle(const SensorMeasurement& measurement) const {
    return std::holds_alternative<core::GnssSolution>(measurement);
}

EstimatorUpdateResult GnssLooselyCoupled::Apply(NominalState& nominal, Eigen::VectorXd& error_state,
                                                Eigen::MatrixXd& covariance, const StateLayout& layout,
                                                const SensorMeasurement& measurement, const UpdateContext& context) {
    if (!context.ltp) return EstimatorUpdateResult::NotInitialized;

    const auto& gnss = std::get<core::GnssSolution>(measurement);
    if (gnss.fix_type == core::GnssFixType::NoFix) return EstimatorUpdateResult::Rejected;
    if (gnss.validity != core::MeasurementValidity::Valid) return EstimatorUpdateResult::Rejected;

    return ApplySolution(nominal, error_state, covariance, layout, gnss, *context.ltp);
}

EstimatorUpdateResult GnssLooselyCoupled::ApplySolution(NominalState& nominal, Eigen::VectorXd& error_state,
                                                        Eigen::MatrixXd& covariance, const StateLayout& layout,
                                                        const core::GnssSolution& gnss,
                                                        const core::LocalTangentPlane& ltp) {
    const int n = layout.TotalSize();
    const Eigen::Matrix3d R_ecef_to_enu = ltp.ecef_to_enu_rotation();

    // ── Convert GNSS to ENU ───────────────────────────────────────────────────
    const Eigen::Vector3d gnss_pos_enu = ltp.EcefToEnu(gnss.position_ecef_m).eigen();
    const Eigen::Vector3d gnss_vel_enu = R_ecef_to_enu * gnss.velocity_ecef_mps.eigen();

    // ── Innovation (6×1) ──────────────────────────────────────────────────────
    Eigen::VectorXd dz(6);
    dz.segment<3>(0) = gnss_pos_enu - nominal.position_enu_m.eigen();
    dz.segment<3>(3) = gnss_vel_enu - nominal.velocity_enu_mps.eigen();

    // ── H matrix (6×n) ────────────────────────────────────────────────────────
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, n);
    const auto& pos_seg = layout.Get(StateSegmentId::Position);
    const auto& vel_seg = layout.Get(StateSegmentId::Velocity);
    H.block(0, pos_seg.offset, 3, 3) = Eigen::Matrix3d::Identity();
    H.block(3, vel_seg.offset, 3, 3) = Eigen::Matrix3d::Identity();

    // ── Measurement noise R (6×6) ─────────────────────────────────────────────
    Eigen::Matrix3d R_pos;
    if (options_.position_sigma_m) {
        const double s = *options_.position_sigma_m;
        R_pos = Eigen::Matrix3d::Identity() * (s * s);
    } else {
        R_pos = R_ecef_to_enu * gnss.position_covariance_ecef_m2 * R_ecef_to_enu.transpose();
    }

    Eigen::Matrix3d R_vel;
    if (options_.velocity_sigma_mps) {
        const double s = *options_.velocity_sigma_mps;
        R_vel = Eigen::Matrix3d::Identity() * (s * s);
    } else {
        R_vel = R_ecef_to_enu * gnss.velocity_covariance_ecef_m2ps2 * R_ecef_to_enu.transpose();
    }

    Eigen::MatrixXd R_meas = Eigen::MatrixXd::Zero(6, 6);
    R_meas.block<3, 3>(0, 0) = R_pos;
    R_meas.block<3, 3>(3, 3) = R_vel;

    // ── Innovation covariance S (6×6) ─────────────────────────────────────────
    const Eigen::MatrixXd S = H * covariance * H.transpose() + R_meas;

    // ── Mahalanobis gate ──────────────────────────────────────────────────────
    if (options_.position_gate > 0.0) {
        const Eigen::Matrix3d S_pos = S.block<3, 3>(0, 0);
        const double mahal = dz.head<3>().transpose() * S_pos.ldlt().solve(dz.head<3>());
        if (mahal > options_.position_gate) return EstimatorUpdateResult::Rejected;
    }

    // ── Kalman gain K (n×6) ───────────────────────────────────────────────────
    const Eigen::MatrixXd K = covariance * H.transpose() * S.inverse();

    // ── Error state update ────────────────────────────────────────────────────
    error_state += K * dz;

    // ── Covariance update — Joseph form ───────────────────────────────────────
    const Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(n, n) - K * H;
    covariance = IKH * covariance * IKH.transpose() + K * R_meas * K.transpose();
    covariance = core::SymmetrizeCovariance(covariance);

    return EstimatorUpdateResult::Accepted;
}

}  // namespace falconguide::estimation::ekf
