#include "falconguide/estimation/backends/ukf/measurement_models/ukf_wheel_odometry.hpp"

#include <variant>

namespace falconguide::estimation::ukf {

UkfWheelOdometry::UkfWheelOdometry(UkfWheelOdometryOptions options) : options_(std::move(options)) {}

bool UkfWheelOdometry::CanHandle(const SensorMeasurement& measurement) const {
    return std::holds_alternative<core::WheelOdometryMeasurement>(measurement);
}

Eigen::VectorXd UkfWheelOdometry::Predict(const NominalState& sigma_state, const UkfUpdateContext& /*ctx*/) const {
    // h(σ) = R^T * v_enu (velocity in body frame)
    const Eigen::Matrix3d R = sigma_state.orientation_body_to_enu.toRotationMatrix();
    return R.transpose() * sigma_state.velocity_enu_mps.eigen();
}

std::optional<Eigen::VectorXd> UkfWheelOdometry::Observe(const SensorMeasurement& measurement,
                                                         const UkfUpdateContext& /*ctx*/) const {
    const auto& odo = std::get<core::WheelOdometryMeasurement>(measurement);
    if (odo.validity != core::MeasurementValidity::Valid) return std::nullopt;
    return odo.linear_velocity_body_mps.eigen();
}

Eigen::MatrixXd UkfWheelOdometry::NoiseCovariance(const SensorMeasurement& /*measurement*/,
                                                  const UkfUpdateContext& /*ctx*/) const {
    const double s = options_.sigma_linear_mps;
    return Eigen::Matrix3d::Identity() * (s * s);
}

}  // namespace falconguide::estimation::ukf
