#include "falconguide/estimation/backends/ekf/measurement_models/range_finder.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/ekf_update.hpp"

#include <variant>

namespace falconguide::estimation::ekf {

RangeFinder::RangeFinder(RangeFinderOptions options) : options_(std::move(options)) {}

bool RangeFinder::CanHandle(const SensorMeasurement& measurement) const {
    return std::holds_alternative<core::RangeFinderMeasurement>(measurement);
}

EstimatorUpdateResult RangeFinder::Apply(NominalState& nominal, Eigen::VectorXd& error_state,
                                         Eigen::MatrixXd& covariance, const StateLayout& layout,
                                         const SensorMeasurement& measurement, const UpdateContext& /*context*/) {
    if (!options_.use_as_altitude) return EstimatorUpdateResult::Rejected;

    const auto& rf = std::get<core::RangeFinderMeasurement>(measurement);
    if (rf.validity != core::MeasurementValidity::Valid) return EstimatorUpdateResult::Rejected;
    if (rf.range_m < 0.0) return EstimatorUpdateResult::Rejected;

    const int n = layout.TotalSize();
    const auto& pos_seg = layout.Get(StateSegmentId::Position);

    const double h_pred = nominal.position_enu_m.eigen().z() - options_.terrain_elevation_m;
    const double sigma = options_.sigma_m.value_or(0.3);

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, n);
    H(0, pos_seg.offset + 2) = 1.0;

    const Eigen::VectorXd dz = Eigen::VectorXd::Constant(1, rf.range_m - h_pred);
    const Eigen::MatrixXd R_meas = Eigen::MatrixXd::Constant(1, 1, sigma * sigma);

    return EkfUpdate(error_state, covariance, H, dz, R_meas, options_.innovation_gate);
}

}  // namespace falconguide::estimation::ekf
