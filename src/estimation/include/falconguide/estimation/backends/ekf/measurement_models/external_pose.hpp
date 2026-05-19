#pragma once

#include "falconguide/estimation/backends/ekf/measurement_models/measurement_model.hpp"

#include <optional>

namespace falconguide::estimation::ekf {

struct ExternalPoseOptions {
  bool use_position{true};
  bool use_orientation{true};

  std::optional<double> position_sigma_m;
  std::optional<double> orientation_sigma_rad;

  // Mahalanobis gate (chi-squared, 3 or 6-DOF).  0 = disabled.
  double innovation_gate{0.0};
};

// ── ExternalPose ──────────────────────────────────────────────────────────────
//
// Full 6-DOF pose update from an external system (motion capture, fiducial,
// UWB-anchored localisation, etc.).
//
// Position  (ECEF → ENU):  H_pos = I₃    (3×3 position block)
// Attitude (body→ECEF → body→ENU via LTP):
//   δθ = LogMapSo3(q_pred^{−1} ⊗ q_obs)
//   H_att = I₃                            (3×3 attitude block)
//
class ExternalPose : public IMeasurementModel {
 public:
  explicit ExternalPose(ExternalPoseOptions options = {});

  [[nodiscard]] bool CanHandle(const SensorMeasurement& measurement) const override;

  EstimatorUpdateResult Apply(
      NominalState& nominal,
      Eigen::VectorXd& error_state,
      Eigen::MatrixXd& covariance,
      const StateLayout& layout,
      const SensorMeasurement& measurement,
      const UpdateContext& context) override;

 private:
  ExternalPoseOptions options_;
};

}  // namespace falconguide::estimation::ekf
