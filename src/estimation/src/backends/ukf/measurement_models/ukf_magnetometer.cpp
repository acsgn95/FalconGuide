#include "falconguide/estimation/backends/ukf/measurement_models/ukf_magnetometer.hpp"

#include <variant>

namespace falconguide::estimation::ukf {

UkfMagnetometer::UkfMagnetometer(UkfMagnetometerOptions options) : options_(std::move(options)) {}

bool UkfMagnetometer::CanHandle(const SensorMeasurement& measurement) const {
  return std::holds_alternative<core::MagnetometerMeasurement>(measurement);
}

Eigen::VectorXd UkfMagnetometer::Predict(
    const NominalState& sigma_state,
    const UkfUpdateContext& /*ctx*/) const {
  // h(σ) = R^T * b_ref_enu (predicted body-frame field)
  const Eigen::Matrix3d R = sigma_state.orientation_body_to_enu.toRotationMatrix();
  return R.transpose() * options_.reference_field_enu_tesla;
}

std::optional<Eigen::VectorXd> UkfMagnetometer::Observe(
    const SensorMeasurement& measurement,
    const UkfUpdateContext& /*ctx*/) const {

  const auto& mag = std::get<core::MagnetometerMeasurement>(measurement);
  if (mag.validity != core::MeasurementValidity::Valid) return std::nullopt;
  return mag.magnetic_field_tesla.eigen();
}

Eigen::MatrixXd UkfMagnetometer::NoiseCovariance(
    const SensorMeasurement& /*measurement*/,
    const UkfUpdateContext& /*ctx*/) const {
  const double s = options_.sigma_tesla.value_or(
      options_.reference_field_enu_tesla.norm() * 0.05);
  return Eigen::Matrix3d::Identity() * (s * s);
}

}  // namespace falconguide::estimation::ukf
