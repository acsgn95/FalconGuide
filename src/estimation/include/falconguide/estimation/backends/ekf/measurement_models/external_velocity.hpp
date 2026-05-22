#pragma once

#include "falconguide/estimation/backends/ekf/measurement_models/measurement_model.hpp"

#include <optional>

namespace falconguide::estimation::ekf {

struct ExternalVelocityOptions {
  std::optional<double> sigma_linear_mps;
  std::optional<double> sigma_angular_radps;

  bool use_angular_rate{false};
  Eigen::Vector3d latest_gyro_radps{0.0, 0.0, 0.0};

  // Mahalanobis gate (chi-squared, 3-DOF).  0 = disabled.
  double innovation_gate{0.0};
};

// ── ExternalVelocity ──────────────────────────────────────────────────────────
//
// Body-frame linear (and optionally angular) velocity update.
// Structurally identical to WheelOdometry but handles ExternalVelocityMeasurement.
//
// h(x) = R_body_to_enu^T * v_enu
//
class ExternalVelocity : public IMeasurementModel {
 public:
  explicit ExternalVelocity(ExternalVelocityOptions options = {});

  [[nodiscard]] bool CanHandle(const SensorMeasurement& measurement) const override;

  EstimatorUpdateResult Apply(
      NominalState& nominal,
      Eigen::VectorXd& error_state,
      Eigen::MatrixXd& covariance,
      const StateLayout& layout,
      const SensorMeasurement& measurement,
      const UpdateContext& context) override;

 private:
  ExternalVelocityOptions options_;
};

}  // namespace falconguide::estimation::ekf
