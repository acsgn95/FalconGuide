#include "falconguide/estimation/backends/ukf/measurement_models/ukf_optical_flow.hpp"

#include <variant>

namespace falconguide::estimation::ukf {

UkfOpticalFlow::UkfOpticalFlow(UkfOpticalFlowOptions options) : options_(std::move(options)) {}

bool UkfOpticalFlow::CanHandle(const SensorMeasurement& measurement) const {
    return std::holds_alternative<core::OpticalFlowMeasurement>(measurement);
}

Eigen::VectorXd UkfOpticalFlow::Predict(const NominalState& sigma_state, const UkfUpdateContext& /*ctx*/) const {
    // h(σ) = [R^T * v_enu].xy (body-frame horizontal velocity)
    const Eigen::Matrix3d R = sigma_state.orientation_body_to_enu.toRotationMatrix();
    const Eigen::Vector3d vb = R.transpose() * sigma_state.velocity_enu_mps.eigen();
    Eigen::VectorXd z(2);
    z(0) = vb.x();
    z(1) = vb.y();
    return z;
}

std::optional<Eigen::VectorXd> UkfOpticalFlow::Observe(const SensorMeasurement& measurement,
                                                       const UkfUpdateContext& /*ctx*/) const {
    const auto& of = std::get<core::OpticalFlowMeasurement>(measurement);
    if (of.validity != core::MeasurementValidity::Valid) return std::nullopt;
    if (!of.integration_time_s || *of.integration_time_s < 1e-6) return std::nullopt;

    const double z = of.ground_distance_m.value_or(options_.fallback_altitude_m);
    if (z < 0.1) return std::nullopt;

    const double dt = *of.integration_time_s;
    Eigen::VectorXd obs(2);
    obs(0) = of.integrated_flow_x_rad / dt * z;
    obs(1) = of.integrated_flow_y_rad / dt * z;
    return obs;
}

Eigen::MatrixXd UkfOpticalFlow::NoiseCovariance(const SensorMeasurement& measurement,
                                                const UkfUpdateContext& /*ctx*/) const {
    const auto& of = std::get<core::OpticalFlowMeasurement>(measurement);
    const double z = of.ground_distance_m.value_or(options_.fallback_altitude_m);
    const double sv = options_.sigma_radps * std::max(z, 0.1);
    return Eigen::Matrix2d::Identity() * (sv * sv);
}

}  // namespace falconguide::estimation::ukf
