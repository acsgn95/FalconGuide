#include "falconguide/estimation/backends/ukf/measurement_models/ukf_airspeed.hpp"

#include <variant>

namespace falconguide::estimation::ukf {

UkfAirspeed::UkfAirspeed(UkfAirspeedOptions options) : options_(std::move(options)) {}

bool UkfAirspeed::CanHandle(const SensorMeasurement& measurement) const {
    return std::holds_alternative<core::AirspeedMeasurement>(measurement);
}

Eigen::VectorXd UkfAirspeed::Predict(const NominalState& sigma_state, const UkfUpdateContext& /*ctx*/) const {
    // h(σ) = ||R^T * (v_enu - v_wind)||
    const Eigen::Matrix3d R = sigma_state.orientation_body_to_enu.toRotationMatrix();
    const Eigen::Vector3d v_air = sigma_state.velocity_enu_mps.eigen() - options_.wind_enu_mps;
    return Eigen::VectorXd::Constant(1, (R.transpose() * v_air).norm());
}

std::optional<Eigen::VectorXd> UkfAirspeed::Observe(const SensorMeasurement& measurement,
                                                    const UkfUpdateContext& /*ctx*/) const {
    const auto& asp = std::get<core::AirspeedMeasurement>(measurement);
    if (asp.validity != core::MeasurementValidity::Valid || asp.airspeed_mps < 0.0) return std::nullopt;
    return Eigen::VectorXd::Constant(1, asp.airspeed_mps);
}

Eigen::MatrixXd UkfAirspeed::NoiseCovariance(const SensorMeasurement& /*measurement*/,
                                             const UkfUpdateContext& /*ctx*/) const {
    return Eigen::MatrixXd::Constant(1, 1, options_.sigma_mps * options_.sigma_mps);
}

}  // namespace falconguide::estimation::ukf
