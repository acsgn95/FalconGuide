#include "falconguide/estimation/backends/ukf/measurement_models/ukf_echo_sounder.hpp"

#include <variant>

namespace falconguide::estimation::ukf {

UkfEchoSounder::UkfEchoSounder(UkfEchoSounderOptions options) : options_(std::move(options)) {}

bool UkfEchoSounder::CanHandle(const SensorMeasurement& measurement) const {
  return std::holds_alternative<core::EchoSounderMeasurement>(measurement);
}

Eigen::VectorXd UkfEchoSounder::Predict(
    const NominalState& sigma_state,
    const UkfUpdateContext& /*ctx*/) const {
  // h(σ) = -z_enu + depth_origin_m  (depth positive downward)
  const double depth = -sigma_state.position_enu_m.eigen().z() + options_.depth_origin_m;
  return Eigen::VectorXd::Constant(1, depth);
}

std::optional<Eigen::VectorXd> UkfEchoSounder::Observe(
    const SensorMeasurement& measurement,
    const UkfUpdateContext& /*ctx*/) const {

  const auto& es = std::get<core::EchoSounderMeasurement>(measurement);
  if (es.validity != core::MeasurementValidity::Valid || es.depth_or_range_m < 0.0)
    return std::nullopt;
  const double sound_speed = es.sound_speed_mps.value_or(options_.sound_speed_mps);
  const double corrected   = es.depth_or_range_m * sound_speed / options_.sound_speed_mps;
  return Eigen::VectorXd::Constant(1, corrected);
}

Eigen::MatrixXd UkfEchoSounder::NoiseCovariance(
    const SensorMeasurement& /*measurement*/,
    const UkfUpdateContext& /*ctx*/) const {
  const double s = options_.sigma_m.value_or(0.2);
  return Eigen::MatrixXd::Constant(1, 1, s * s);
}

}  // namespace falconguide::estimation::ukf
