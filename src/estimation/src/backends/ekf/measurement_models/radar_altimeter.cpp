#include "falconguide/estimation/backends/ekf/measurement_models/radar_altimeter.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/ekf_update.hpp"

#include <variant>

namespace falconguide::estimation::ekf {

RadarAltimeter::RadarAltimeter(RadarAltimeterOptions options) : options_(std::move(options)) {}

bool RadarAltimeter::CanHandle(const SensorMeasurement& measurement) const {
    return std::holds_alternative<core::RadarAltimeterMeasurement>(measurement);
}

EstimatorUpdateResult RadarAltimeter::Apply(NominalState& nominal, Eigen::VectorXd& error_state,
                                            Eigen::MatrixXd& covariance, const StateLayout& layout,
                                            const SensorMeasurement& measurement, const UpdateContext& /*context*/) {
    const auto& ralt = std::get<core::RadarAltimeterMeasurement>(measurement);
    if (ralt.validity != core::MeasurementValidity::Valid) return EstimatorUpdateResult::Rejected;
    if (ralt.surface == core::RadarAltimeterSurface::Invalid) return EstimatorUpdateResult::Rejected;
    if (ralt.range_m < 0.0) return EstimatorUpdateResult::Rejected;

    const int n = layout.TotalSize();
    const auto& pos_seg = layout.Get(StateSegmentId::Position);

    const double h_pred = nominal.position_enu_m.eigen().z() - options_.terrain_elevation_m;
    const double sigma = options_.sigma_m.value_or(0.5);

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, n);
    H(0, pos_seg.offset + 2) = 1.0;

    const Eigen::VectorXd dz = Eigen::VectorXd::Constant(1, ralt.range_m - h_pred);
    const Eigen::MatrixXd R_meas = Eigen::MatrixXd::Constant(1, 1, sigma * sigma);

    return EkfUpdate(error_state, covariance, H, dz, R_meas, options_.innovation_gate);
}

}  // namespace falconguide::estimation::ekf
