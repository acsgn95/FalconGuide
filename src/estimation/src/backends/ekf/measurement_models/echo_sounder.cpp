#include "falconguide/estimation/backends/ekf/measurement_models/echo_sounder.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/ekf_update.hpp"

#include <variant>

namespace falconguide::estimation::ekf {

EchoSounder::EchoSounder(EchoSounderOptions options) : options_(std::move(options)) {}

bool EchoSounder::CanHandle(const SensorMeasurement& measurement) const {
    return std::holds_alternative<core::EchoSounderMeasurement>(measurement);
}

EstimatorUpdateResult EchoSounder::Apply(NominalState& nominal, Eigen::VectorXd& error_state,
                                         Eigen::MatrixXd& covariance, const StateLayout& layout,
                                         const SensorMeasurement& measurement, const UpdateContext& /*context*/) {
    const auto& es = std::get<core::EchoSounderMeasurement>(measurement);
    if (es.validity != core::MeasurementValidity::Valid) return EstimatorUpdateResult::Rejected;
    if (es.depth_or_range_m < 0.0) return EstimatorUpdateResult::Rejected;

    // Apply sound speed correction if sensor provided it.
    const double sound_speed = es.sound_speed_mps.value_or(options_.sound_speed_mps);
    const double corrected_depth = es.depth_or_range_m * sound_speed / options_.sound_speed_mps;

    const int n = layout.TotalSize();
    const auto& pos_seg = layout.Get(StateSegmentId::Position);

    // h(x) = -p_enu.z + depth_origin_m  (depth positive downward)
    const double h_pred = -nominal.position_enu_m.eigen().z() + options_.depth_origin_m;
    const double sigma = options_.sigma_m.value_or(0.2);

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, n);
    H(0, pos_seg.offset + 2) = -1.0;  // depth = -z_enu

    const Eigen::VectorXd dz = Eigen::VectorXd::Constant(1, corrected_depth - h_pred);
    const Eigen::MatrixXd R_meas = Eigen::MatrixXd::Constant(1, 1, sigma * sigma);

    return EkfUpdate(error_state, covariance, H, dz, R_meas, options_.innovation_gate);
}

}  // namespace falconguide::estimation::ekf
