#include "falconguide/estimation/backends/ukf/measurement_models/ukf_dvl.hpp"

#include <variant>

namespace falconguide::estimation::ukf {

UkfDvl::UkfDvl(UkfDvlOptions options) : options_(std::move(options)) {}

bool UkfDvl::CanHandle(const SensorMeasurement& measurement) const {
  return std::holds_alternative<core::DvlMeasurement>(measurement);
}

Eigen::VectorXd UkfDvl::Predict(
    const NominalState& sigma_state,
    const UkfUpdateContext& /*ctx*/) const {
  // h(σ) = R_dvl_to_body^T * R_body_to_enu^T * v_enu
  const Eigen::Matrix3d R = sigma_state.orientation_body_to_enu.toRotationMatrix();
  return options_.dvl_to_body_rotation.transpose() *
         R.transpose() * sigma_state.velocity_enu_mps.eigen();
}

std::optional<Eigen::VectorXd> UkfDvl::Observe(
    const SensorMeasurement& measurement,
    const UkfUpdateContext& /*ctx*/) const {

  const auto& dvl = std::get<core::DvlMeasurement>(measurement);
  if (dvl.validity != core::MeasurementValidity::Valid) return std::nullopt;
  if (options_.bottom_track_only && dvl.reference != core::DvlReference::BottomTrack)
    return std::nullopt;
  return dvl.velocity_mps.eigen();
}

Eigen::MatrixXd UkfDvl::NoiseCovariance(
    const SensorMeasurement& measurement,
    const UkfUpdateContext& /*ctx*/) const {

  const auto& dvl = std::get<core::DvlMeasurement>(measurement);
  if (dvl.velocity_covariance_m2ps2.trace() > 1e-12)
    return dvl.velocity_covariance_m2ps2;
  const double s = options_.sigma_mps.value_or(0.05);
  return Eigen::Matrix3d::Identity() * (s * s);
}

}  // namespace falconguide::estimation::ukf
