#include "falconguide/estimation/backends/ukf/measurement_models/ukf_external_pose.hpp"

#include "falconguide/core/math.hpp"

#include <variant>

namespace falconguide::estimation::ukf {

UkfExternalPose::UkfExternalPose(UkfExternalPoseOptions options) : options_(std::move(options)) {}

bool UkfExternalPose::CanHandle(const SensorMeasurement& measurement) const {
    return std::holds_alternative<core::ExternalPoseMeasurement>(measurement);
}

int UkfExternalPose::MeasurementDim(const SensorMeasurement& /*measurement*/) const {
    return (options_.use_position ? 3 : 0) + (options_.use_orientation ? 3 : 0);
}

Eigen::VectorXd UkfExternalPose::Predict(const NominalState& sigma_state, const UkfUpdateContext& /*ctx*/) const {
    const int m = (options_.use_position ? 3 : 0) + (options_.use_orientation ? 3 : 0);
    Eigen::VectorXd z(m);
    int row = 0;
    if (options_.use_position) {
        z.segment<3>(row) = sigma_state.position_enu_m.eigen();
        row += 3;
    }
    if (options_.use_orientation) {
        z.segment<3>(row) = core::LogMapSo3(sigma_state.orientation_body_to_enu);
    }
    return z;
}

std::optional<Eigen::VectorXd> UkfExternalPose::Observe(const SensorMeasurement& measurement,
                                                        const UkfUpdateContext& ctx) const {
    const auto& ep = std::get<core::ExternalPoseMeasurement>(measurement);
    if (ep.validity != core::MeasurementValidity::Valid || !ctx.ltp) return std::nullopt;

    const int m = (options_.use_position ? 3 : 0) + (options_.use_orientation ? 3 : 0);
    Eigen::VectorXd z(m);
    int row = 0;

    if (options_.use_position) {
        z.segment<3>(row) = ctx.ltp->EcefToEnu(ep.position_ecef_m).eigen();
        row += 3;
    }
    if (options_.use_orientation) {
        const Eigen::Matrix3d R_ecef_to_enu = ctx.ltp->ecef_to_enu_rotation();
        const Eigen::Quaterniond q_obs = (Eigen::Quaterniond(R_ecef_to_enu) * ep.orientation_body_to_ecef).normalized();
        z.segment<3>(row) = core::LogMapSo3(q_obs);
    }
    return z;
}

Eigen::MatrixXd UkfExternalPose::NoiseCovariance(const SensorMeasurement& measurement,
                                                 const UkfUpdateContext& ctx) const {
    const auto& ep = std::get<core::ExternalPoseMeasurement>(measurement);
    const int m = (options_.use_position ? 3 : 0) + (options_.use_orientation ? 3 : 0);
    Eigen::MatrixXd R_meas = Eigen::MatrixXd::Zero(m, m);
    int row = 0;

    if (options_.use_position) {
        if (options_.position_sigma_m) {
            const double s = *options_.position_sigma_m;
            R_meas.block<3, 3>(row, row) = Eigen::Matrix3d::Identity() * (s * s);
        } else {
            const Eigen::Matrix3d Re = ctx.ltp ? ctx.ltp->ecef_to_enu_rotation() : Eigen::Matrix3d::Identity();
            R_meas.block<3, 3>(row, row) = Re * ep.covariance.block<3, 3>(0, 0) * Re.transpose();
            if (R_meas.block<3, 3>(row, row).trace() < 1e-12)
                R_meas.block<3, 3>(row, row) = Eigen::Matrix3d::Identity() * 4.0;
        }
        row += 3;
    }
    if (options_.use_orientation) {
        const double s = options_.orientation_sigma_rad.value_or(0.02);
        R_meas.block<3, 3>(row, row) = Eigen::Matrix3d::Identity() * (s * s);
    }
    return R_meas;
}

}  // namespace falconguide::estimation::ukf
