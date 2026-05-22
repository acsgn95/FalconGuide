#pragma once

#include "falconguide/estimation/backends/ekf/measurement_models/measurement_model.hpp"

#include <optional>

namespace falconguide::estimation::ekf {

struct ExternalOdometryOptions {
  bool use_position{true};
  bool use_velocity{true};
  bool use_orientation{true};

  std::optional<double> position_sigma_m;
  std::optional<double> velocity_sigma_mps;
  std::optional<double> orientation_sigma_rad;

  // Mahalanobis gate (chi-squared, applied per sub-block).  0 = disabled.
  double innovation_gate{0.0};
};

// ── ExternalOdometry ──────────────────────────────────────────────────────────
//
// Full 9-DOF pose+velocity update (e.g. from an external SLAM system).
// Selectively applies position, velocity, and attitude sub-updates.
//
// Combines the logic of ExternalPose (position + attitude) and GNSS loosely
// coupled (velocity) into one model that handles ExternalOdometryMeasurement.
//
class ExternalOdometry : public IMeasurementModel {
 public:
  explicit ExternalOdometry(ExternalOdometryOptions options = {});

  [[nodiscard]] bool CanHandle(const SensorMeasurement& measurement) const override;

  EstimatorUpdateResult Apply(
      NominalState& nominal,
      Eigen::VectorXd& error_state,
      Eigen::MatrixXd& covariance,
      const StateLayout& layout,
      const SensorMeasurement& measurement,
      const UpdateContext& context) override;

 private:
  ExternalOdometryOptions options_;
};

}  // namespace falconguide::estimation::ekf
