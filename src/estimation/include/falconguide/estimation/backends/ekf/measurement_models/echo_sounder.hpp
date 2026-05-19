#pragma once

#include "falconguide/estimation/backends/ekf/measurement_models/measurement_model.hpp"

#include <optional>

namespace falconguide::estimation::ekf {

struct EchoSounderOptions {
  // Sound speed correction factor.  depth_corrected = depth_raw * sound_speed / 1500.
  double sound_speed_mps{1500.0};

  // Depth below the LTP z-origin (positive down).
  // h(x) = -p_enu.z + depth_origin_m
  double depth_origin_m{0.0};

  std::optional<double> sigma_m;

  // Mahalanobis gate (chi-squared, 1-DOF).  0 = disabled.
  double innovation_gate{0.0};
};

// ── EchoSounder ───────────────────────────────────────────────────────────────
//
// Scalar depth-below-surface update (underwater navigation).
//
// h(x) = -p_enu.z + depth_origin_m    (positive depth = below origin)
//
// H: [0,0,-1, 0,...] at position z column.
//
class EchoSounder : public IMeasurementModel {
 public:
  explicit EchoSounder(EchoSounderOptions options = {});

  [[nodiscard]] bool CanHandle(const SensorMeasurement& measurement) const override;

  EstimatorUpdateResult Apply(
      NominalState& nominal,
      Eigen::VectorXd& error_state,
      Eigen::MatrixXd& covariance,
      const StateLayout& layout,
      const SensorMeasurement& measurement,
      const UpdateContext& context) override;

 private:
  EchoSounderOptions options_;
};

}  // namespace falconguide::estimation::ekf
