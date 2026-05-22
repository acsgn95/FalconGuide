#include "falconguide/estimation/backends/ekf/measurement_models/optical_flow.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/ekf_update.hpp"

#include "falconguide/core/math.hpp"

#include <variant>

namespace falconguide::estimation::ekf {

OpticalFlow::OpticalFlow(OpticalFlowOptions options) : options_(std::move(options)) {}

bool OpticalFlow::CanHandle(const SensorMeasurement& measurement) const {
    return std::holds_alternative<core::OpticalFlowMeasurement>(measurement);
}

EstimatorUpdateResult OpticalFlow::Apply(NominalState& nominal, Eigen::VectorXd& error_state,
                                         Eigen::MatrixXd& covariance, const StateLayout& layout,
                                         const SensorMeasurement& measurement, const UpdateContext& /*context*/) {
    const auto& of = std::get<core::OpticalFlowMeasurement>(measurement);
    if (of.validity != core::MeasurementValidity::Valid) return EstimatorUpdateResult::Rejected;
    if (!of.integration_time_s || *of.integration_time_s < 1e-6) return EstimatorUpdateResult::Rejected;

    const double z = of.ground_distance_m.value_or(options_.fallback_altitude_m);
    if (z < 0.1) return EstimatorUpdateResult::Rejected;

    const double dt = *of.integration_time_s;

    // Convert integrated flow to body-frame velocity
    const double v_body_x = of.integrated_flow_x_rad / dt * z;
    const double v_body_y = of.integrated_flow_y_rad / dt * z;

    const int n = layout.TotalSize();
    const auto& vel_seg = layout.Get(StateSegmentId::Velocity);
    const auto& att_seg = layout.Get(StateSegmentId::Attitude);

    const Eigen::Matrix3d R = nominal.orientation_body_to_enu.toRotationMatrix();
    const Eigen::Vector3d v_body = R.transpose() * nominal.velocity_enu_mps.eigen();

    // h(x) = [R^T * v_enu].xy (2-component body horizontal velocity)
    Eigen::VectorXd dz(2);
    dz(0) = v_body_x - v_body.x();
    dz(1) = v_body_y - v_body.y();

    // H (2×n): top 2 rows of WheelOdometry H
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, n);
    H.block<2, 3>(0, vel_seg.offset) = R.transpose().topRows<2>();
    H.block<2, 3>(0, att_seg.offset) = core::SkewSymmetric(v_body).topRows<2>();

    // Noise: sigma in rad/s, scaled to velocity
    const double sv = options_.sigma_radps * z;
    const Eigen::MatrixXd R_meas = Eigen::Matrix2d::Identity() * (sv * sv);

    return EkfUpdate(error_state, covariance, H, dz, R_meas, options_.innovation_gate);
}

}  // namespace falconguide::estimation::ekf
