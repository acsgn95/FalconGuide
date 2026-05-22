#pragma once

#include "falconguide/estimation/backends/ekf/measurement_models/measurement_model.hpp"

#include <optional>

namespace falconguide::estimation::ekf {

struct OpticalFlowOptions {
  // Fallback ground distance when measurement.ground_distance_m is absent (m).
  double fallback_altitude_m{10.0};

  // Noise on each flow component (rad/s) before altitude scaling.
  double sigma_radps{0.01};

  // Mahalanobis gate (chi-squared, 2-DOF).  0 = disabled.
  double innovation_gate{0.0};
};

// ── OpticalFlow ───────────────────────────────────────────────────────────────
//
// 2-axis body horizontal velocity update derived from integrated optical flow.
//
// Conversion:
//   v_body_x = flow_x / dt * z
//   v_body_y = flow_y / dt * z
//
// where z = ground_distance_m (from measurement or fallback).
//
// Measurement function (same as WheelOdometry, 2 components):
//   h(x) = [R^T * v_enu].xy
//
// H_vel_xy = R^T.topRows(2)                          (2×3 velocity block)
// H_att_xy = SkewSymmetric(R^T * v_enu).topRows(2)   (2×3 attitude block)
//
class OpticalFlow : public IMeasurementModel {
 public:
  explicit OpticalFlow(OpticalFlowOptions options = {});

  [[nodiscard]] bool CanHandle(const SensorMeasurement& measurement) const override;

  EstimatorUpdateResult Apply(
      NominalState& nominal,
      Eigen::VectorXd& error_state,
      Eigen::MatrixXd& covariance,
      const StateLayout& layout,
      const SensorMeasurement& measurement,
      const UpdateContext& context) override;

 private:
  OpticalFlowOptions options_;
};

}  // namespace falconguide::estimation::ekf
