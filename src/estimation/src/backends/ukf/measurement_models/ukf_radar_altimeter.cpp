#include "falconguide/estimation/backends/ukf/measurement_models/ukf_radar_altimeter.hpp"

#include <variant>

namespace falconguide::estimation::ukf {

UkfRadarAltimeter::UkfRadarAltimeter(UkfRadarAltimeterOptions options) : options_(std::move(options)) {}

bool UkfRadarAltimeter::CanHandle(const SensorMeasurement& measurement) const {
    return std::holds_alternative<core::RadarAltimeterMeasurement>(measurement);
}

Eigen::VectorXd UkfRadarAltimeter::Predict(const NominalState& sigma_state, const UkfUpdateContext& /*ctx*/) const {
    // h(σ) = z_enu - terrain_elevation_m
    const double alt = sigma_state.position_enu_m.eigen().z() - options_.terrain_elevation_m;
    return Eigen::VectorXd::Constant(1, alt);
}

std::optional<Eigen::VectorXd> UkfRadarAltimeter::Observe(const SensorMeasurement& measurement,
                                                          const UkfUpdateContext& /*ctx*/) const {
    const auto& ralt = std::get<core::RadarAltimeterMeasurement>(measurement);
    if (ralt.validity == core::MeasurementValidity::Valid && ralt.surface != core::RadarAltimeterSurface::Invalid &&
        ralt.range_m >= 0.0) {
        return Eigen::VectorXd::Constant(1, ralt.range_m);
    }
    return std::nullopt;
}

Eigen::MatrixXd UkfRadarAltimeter::NoiseCovariance(const SensorMeasurement& /*measurement*/,
                                                   const UkfUpdateContext& /*ctx*/) const {
    const double s = options_.sigma_m.value_or(0.5);
    return Eigen::MatrixXd::Constant(1, 1, s * s);
}

}  // namespace falconguide::estimation::ukf
