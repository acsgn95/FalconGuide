#pragma once

#include "falconguide/core/coordinates.hpp"
#include "falconguide/estimation/backends/ukf/ukf_state.hpp"
#include "falconguide/estimation/estimator_interface.hpp"

#include <Eigen/Core>
#include <optional>

namespace falconguide::estimation::ukf {

// ── UkfUpdateContext ──────────────────────────────────────────────────────────
struct UkfUpdateContext {
  const core::LocalTangentPlane* ltp{nullptr};
};

// ── IUkfMeasurementModel ──────────────────────────────────────────────────────
//
// Plugin interface for UKF measurement update steps.
//
// The UKF calls Predict() once per sigma point (2n+1 = 31 times for a 15-state
// filter), passing the full NominalState corresponding to that sigma point.
// No Jacobians are needed — the nonlinear measurement function is evaluated
// directly, which makes implementing new sensor types significantly easier than
// the EKF equivalent.
//
// Adding a new sensor:
//   1. Implement IUkfMeasurementModel.
//   2. Register with UkfEstimator::RegisterMeasurementModel().
//   3. Nothing else changes.
//
class IUkfMeasurementModel {
 public:
  virtual ~IUkfMeasurementModel() = default;

  [[nodiscard]] virtual std::string_view Name() const { return "Unknown"; }

  [[nodiscard]] virtual bool CanHandle(const SensorMeasurement& measurement) const = 0;

  // Dimension of the predicted measurement vector for this sensor.
  [[nodiscard]] virtual int MeasurementDim(const SensorMeasurement& measurement) const = 0;

  // Nonlinear measurement function h(xᵢ).
  // Called once per sigma point during the unscented transform.
  [[nodiscard]] virtual Eigen::VectorXd Predict(
      const NominalState& sigma_state,
      const UkfUpdateContext& ctx) const = 0;

  // Extract the actual observation z from the measurement.
  // Returns nullopt if the measurement should be rejected (e.g. NoFix).
  [[nodiscard]] virtual std::optional<Eigen::VectorXd> Observe(
      const SensorMeasurement& measurement,
      const UkfUpdateContext& ctx) const = 0;

  // Measurement noise covariance R (measurement_dim × measurement_dim).
  [[nodiscard]] virtual Eigen::MatrixXd NoiseCovariance(
      const SensorMeasurement& measurement,
      const UkfUpdateContext& ctx) const = 0;

  // Innovation z_obs − z_pred.  Override to handle angle wrapping.
  [[nodiscard]] virtual Eigen::VectorXd Innovation(
      const Eigen::VectorXd& z_obs,
      const Eigen::VectorXd& z_pred) const {
    return z_obs - z_pred;
  }
};

}  // namespace falconguide::estimation::ukf
