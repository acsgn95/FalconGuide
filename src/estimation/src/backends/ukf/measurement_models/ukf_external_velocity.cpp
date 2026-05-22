#include "falconguide/estimation/backends/ukf/measurement_models/ukf_external_velocity.hpp"

#include <variant>

namespace falconguide::estimation::ukf {

UkfExternalVelocity::UkfExternalVelocity(UkfExternalVelocityOptions options) : options_(std::move(options)) {}

bool UkfExternalVelocity::CanHandle(const SensorMeasurement& measurement) const {
    return std::holds_alternative<core::ExternalVelocityMeasurement>(measurement);
}

Eigen::VectorXd UkfExternalVelocity::Predict(const NominalState& sigma_state, const UkfUpdateContext& /*ctx*/) const {
    // h(σ) = R^T * v_enu  (body-frame linear velocity)
    const Eigen::Matrix3d R = sigma_state.orientation_body_to_enu.toRotationMatrix();
    return R.transpose() * sigma_state.velocity_enu_mps.eigen();
}

std::optional<Eigen::VectorXd> UkfExternalVelocity::Observe(const SensorMeasurement& measurement,
                                                            const UkfUpdateContext& /*ctx*/) const {
    const auto& ev = std::get<core::ExternalVelocityMeasurement>(measurement);
    if (ev.validity != core::MeasurementValidity::Valid) return std::nullopt;
    return ev.linear_velocity_body_mps.eigen();
}

Eigen::MatrixXd UkfExternalVelocity::NoiseCovariance(const SensorMeasurement& measurement,
                                                     const UkfUpdateContext& /*ctx*/) const {
    const auto& ev = std::get<core::ExternalVelocityMeasurement>(measurement);
    if (ev.covariance.block<3, 3>(0, 0).trace() > 1e-12) return ev.covariance.block<3, 3>(0, 0);
    const double s = options_.sigma_linear_mps.value_or(0.1);
    return Eigen::Matrix3d::Identity() * (s * s);
}

}  // namespace falconguide::estimation::ukf
