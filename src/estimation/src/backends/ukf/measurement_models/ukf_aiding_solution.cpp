#include "falconguide/estimation/backends/ukf/measurement_models/ukf_aiding_solution.hpp"

#include "falconguide/core/math.hpp"

#include <variant>

namespace falconguide::estimation::ukf {

UkfAidingSolution::UkfAidingSolution(UkfAidingSolutionOptions options) : options_(std::move(options)) {}

bool UkfAidingSolution::CanHandle(const SensorMeasurement& measurement) const {
    return std::holds_alternative<core::AidingSolution>(measurement);
}

std::tuple<bool, bool, bool> UkfAidingSolution::ActiveBlocks(const core::AidingSolution& aid) {
    return {aid.position_ecef_m.has_value(), aid.velocity_ecef_mps.has_value(),
            aid.orientation_body_to_ecef.has_value() && aid.orientation_validity.any()};
}

int UkfAidingSolution::MeasurementDim(const SensorMeasurement& measurement) const {
    const auto& aid = std::get<core::AidingSolution>(measurement);
    auto [has_pos, has_vel, has_att] = ActiveBlocks(aid);
    return (has_pos ? 3 : 0) + (has_vel ? 3 : 0) + (has_att ? 3 : 0);
}

Eigen::VectorXd UkfAidingSolution::Predict(const NominalState& sigma_state, const UkfUpdateContext& ctx) const {
    // We don't know which blocks are active without the measurement, so predict
    // the full 9-vector and Observe() will select the relevant rows.
    // This is called from the general UKF loop which already knows the correct dim.
    // Build pos(3) + vel(3) + att(3) — Observe masks unused rows.
    Eigen::VectorXd z(9);
    z.segment<3>(0) = sigma_state.position_enu_m.eigen();
    z.segment<3>(3) = sigma_state.velocity_enu_mps.eigen();
    z.segment<3>(6) = core::LogMapSo3(sigma_state.orientation_body_to_enu);
    return z;
}

std::optional<Eigen::VectorXd> UkfAidingSolution::Observe(const SensorMeasurement& measurement,
                                                          const UkfUpdateContext& ctx) const {
    const auto& aid = std::get<core::AidingSolution>(measurement);
    if (aid.validity != core::MeasurementValidity::Valid) return std::nullopt;
    if (aid.match_score && *aid.match_score < options_.min_match_score) return std::nullopt;
    if (!ctx.ltp) return std::nullopt;

    const Eigen::Matrix3d R_ecef_to_enu = ctx.ltp->ecef_to_enu_rotation();
    auto [has_pos, has_vel, has_att] = ActiveBlocks(aid);
    const int m = (has_pos ? 3 : 0) + (has_vel ? 3 : 0) + (has_att ? 3 : 0);
    if (m == 0) return std::nullopt;

    Eigen::VectorXd z(m);
    int row = 0;
    if (has_pos) {
        z.segment<3>(row) = ctx.ltp->EcefToEnu(*aid.position_ecef_m).eigen();
        row += 3;
    }
    if (has_vel) {
        z.segment<3>(row) = R_ecef_to_enu * aid.velocity_ecef_mps->eigen();
        row += 3;
    }
    if (has_att) {
        const Eigen::Quaterniond q_ecef_to_enu(R_ecef_to_enu);
        const Eigen::Quaterniond q_obs = (q_ecef_to_enu * *aid.orientation_body_to_ecef).normalized();
        z.segment<3>(row) = core::LogMapSo3(q_obs);
    }
    return z;
}

Eigen::MatrixXd UkfAidingSolution::NoiseCovariance(const SensorMeasurement& measurement,
                                                   const UkfUpdateContext& ctx) const {
    const auto& aid = std::get<core::AidingSolution>(measurement);
    auto [has_pos, has_vel, has_att] = ActiveBlocks(aid);
    const int m = (has_pos ? 3 : 0) + (has_vel ? 3 : 0) + (has_att ? 3 : 0);

    Eigen::MatrixXd R_meas = Eigen::MatrixXd::Zero(m, m);
    int row = 0;
    const Eigen::Matrix3d R_ecef_to_enu = ctx.ltp ? ctx.ltp->ecef_to_enu_rotation() : Eigen::Matrix3d::Identity();
    if (has_pos) {
        const double s = options_.position_sigma_m.value_or(0.0);
        if (s > 0.0) {
            R_meas.block<3, 3>(row, row) = Eigen::Matrix3d::Identity() * (s * s);
        } else {
            R_meas.block<3, 3>(row, row) = R_ecef_to_enu * aid.position_covariance_ecef_m2 * R_ecef_to_enu.transpose();
            if (R_meas.block<3, 3>(row, row).trace() < 1e-12)
                R_meas.block<3, 3>(row, row) = Eigen::Matrix3d::Identity() * 100.0;
        }
        row += 3;
    }
    if (has_vel) {
        const double s = options_.velocity_sigma_mps.value_or(0.0);
        if (s > 0.0) {
            R_meas.block<3, 3>(row, row) = Eigen::Matrix3d::Identity() * (s * s);
        } else {
            R_meas.block<3, 3>(row, row) =
                R_ecef_to_enu * aid.velocity_covariance_ecef_mps2 * R_ecef_to_enu.transpose();
            if (R_meas.block<3, 3>(row, row).trace() < 1e-12)
                R_meas.block<3, 3>(row, row) = Eigen::Matrix3d::Identity() * 1.0;
        }
        row += 3;
    }
    if (has_att) {
        const double s = options_.attitude_sigma_rad.value_or(0.05);
        R_meas.block<3, 3>(row, row) = Eigen::Matrix3d::Identity() * (s * s);
    }
    return R_meas;
}

}  // namespace falconguide::estimation::ukf
