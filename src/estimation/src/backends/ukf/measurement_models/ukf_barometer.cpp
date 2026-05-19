#include "falconguide/estimation/backends/ukf/measurement_models/ukf_barometer.hpp"

#include <cmath>
#include <variant>

namespace falconguide::estimation::ukf {

static constexpr double kIsaExponent = 1.0 / 5.25588;

static double PressureToAltitude(double pressure_pa, double sea_level_pa) {
  return 44330.77 * (1.0 - std::pow(pressure_pa / sea_level_pa, kIsaExponent));
}

UkfBarometer::UkfBarometer(UkfBarometerOptions options) : options_(std::move(options)) {}

bool UkfBarometer::CanHandle(const SensorMeasurement& measurement) const {
  return std::holds_alternative<core::BarometerMeasurement>(measurement);
}

Eigen::VectorXd UkfBarometer::Predict(
    const NominalState& sigma_state,
    const UkfUpdateContext& /*ctx*/) const {
  // h(σ) = z_enu (altitude above LTP origin)
  return Eigen::VectorXd::Constant(1, sigma_state.position_enu_m.eigen().z());
}

std::optional<Eigen::VectorXd> UkfBarometer::Observe(
    const SensorMeasurement& measurement,
    const UkfUpdateContext& /*ctx*/) const {

  const auto& baro = std::get<core::BarometerMeasurement>(measurement);
  if (baro.validity != core::MeasurementValidity::Valid) return std::nullopt;

  double z_obs;
  if (baro.altitude_m) {
    z_obs = *baro.altitude_m;
  } else if (baro.pressure_pa > 0.0) {
    z_obs = PressureToAltitude(baro.pressure_pa, options_.isa_sea_level_pressure_pa);
  } else {
    return std::nullopt;
  }
  return Eigen::VectorXd::Constant(1, z_obs);
}

Eigen::MatrixXd UkfBarometer::NoiseCovariance(
    const SensorMeasurement& /*measurement*/,
    const UkfUpdateContext& /*ctx*/) const {
  const double s = options_.altitude_sigma_m.value_or(2.0);
  return Eigen::MatrixXd::Constant(1, 1, s * s);
}

}  // namespace falconguide::estimation::ukf
