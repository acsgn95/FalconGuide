#include "falconguide/estimation/backends/ukf/measurement_models/ukf_gnss_loosely_coupled.hpp"

#include <variant>

namespace falconguide::estimation::ukf {

UkfGnssLooselyCoupled::UkfGnssLooselyCoupled(UkfGnssLooselyCoupledOptions options) : options_(std::move(options)) {}

bool UkfGnssLooselyCoupled::CanHandle(const SensorMeasurement& measurement) const {
    return std::holds_alternative<core::GnssSolution>(measurement);
}

int UkfGnssLooselyCoupled::MeasurementDim(const SensorMeasurement&) const {
    return 6;  // [p_enu(3); v_enu(3)]
}

// ── Predict ───────────────────────────────────────────────────────────────────
//
// h(xᵢ) = [position_enu ; velocity_enu]
//
// Because sigma points are already expressed in ENU, this is trivial —
// no Jacobian, no coordinate rotation, no linearisation.
//
Eigen::VectorXd UkfGnssLooselyCoupled::Predict(const NominalState& sigma_state, const UkfUpdateContext& /*ctx*/) const {
    Eigen::VectorXd z(6);
    z.segment<3>(0) = sigma_state.position_enu_m.eigen();
    z.segment<3>(3) = sigma_state.velocity_enu_mps.eigen();
    return z;
}

// ── Observe ───────────────────────────────────────────────────────────────────
std::optional<Eigen::VectorXd> UkfGnssLooselyCoupled::Observe(const SensorMeasurement& measurement,
                                                              const UkfUpdateContext& ctx) const {
    if (!ctx.ltp) return std::nullopt;
    const auto& gnss = std::get<core::GnssSolution>(measurement);
    if (gnss.fix_type == core::GnssFixType::NoFix) return std::nullopt;
    if (gnss.validity != core::MeasurementValidity::Valid) return std::nullopt;

    const Eigen::Matrix3d R_ecef_to_enu = ctx.ltp->ecef_to_enu_rotation();

    Eigen::VectorXd z(6);
    z.segment<3>(0) = ctx.ltp->EcefToEnu(gnss.position_ecef_m).eigen();
    z.segment<3>(3) = R_ecef_to_enu * gnss.velocity_ecef_mps.eigen();
    return z;
}

// ── NoiseCovariance ───────────────────────────────────────────────────────────
Eigen::MatrixXd UkfGnssLooselyCoupled::NoiseCovariance(const SensorMeasurement& measurement,
                                                       const UkfUpdateContext& ctx) const {
    const auto& gnss = std::get<core::GnssSolution>(measurement);
    const Eigen::Matrix3d R = ctx.ltp ? ctx.ltp->ecef_to_enu_rotation() : Eigen::Matrix3d::Identity();

    Eigen::Matrix3d R_pos;
    if (options_.position_sigma_m) {
        const double s = *options_.position_sigma_m;
        R_pos = Eigen::Matrix3d::Identity() * (s * s);
    } else {
        R_pos = R * gnss.position_covariance_ecef_m2 * R.transpose();
    }

    Eigen::Matrix3d R_vel;
    if (options_.velocity_sigma_mps) {
        const double s = *options_.velocity_sigma_mps;
        R_vel = Eigen::Matrix3d::Identity() * (s * s);
    } else {
        R_vel = R * gnss.velocity_covariance_ecef_m2ps2 * R.transpose();
    }

    Eigen::MatrixXd R_meas = Eigen::MatrixXd::Zero(6, 6);
    R_meas.block<3, 3>(0, 0) = R_pos;
    R_meas.block<3, 3>(3, 3) = R_vel;
    return R_meas;
}

}  // namespace falconguide::estimation::ukf
