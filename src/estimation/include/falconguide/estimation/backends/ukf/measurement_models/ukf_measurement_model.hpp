#pragma once

/**
 * @file ukf_measurement_model.hpp
 * @brief Plugin interface for UKF nonlinear measurement models.
 */

#include "falconguide/core/coordinates.hpp"
#include "falconguide/estimation/backends/ukf/ukf_state.hpp"
#include "falconguide/estimation/estimator_interface.hpp"

#include <Eigen/Core>
#include <optional>

namespace falconguide::estimation::ukf {

// ── UkfUpdateContext
// ──────────────────────────────────────────────────────────
struct UkfUpdateContext {
  const core::LocalTangentPlane *ltp{
      nullptr}; ///< Local tangent plane, or nullptr before initialization.
};

// ── IUkfMeasurementModel
// ──────────────────────────────────────────────────────
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
  /// @brief Virtual destructor for interface use.
  virtual ~IUkfMeasurementModel() = default;

  /// @brief Human-readable model name used in reports.
  [[nodiscard]] virtual std::string_view Name() const { return "Unknown"; }

  /// @brief Returns true when this model can process a measurement variant.
  [[nodiscard]] virtual bool
  CanHandle(const SensorMeasurement &measurement) const = 0;

  /// @brief Dimension of the predicted measurement vector for this sensor.
  [[nodiscard]] virtual int
  MeasurementDim(const SensorMeasurement &measurement) const = 0;

  /// @brief Nonlinear measurement function h(x_i), evaluated for each sigma
  /// point.
  [[nodiscard]] virtual Eigen::VectorXd
  Predict(const NominalState &sigma_state,
          const UkfUpdateContext &ctx) const = 0;

  /// @brief Extracts the actual observation vector from a measurement.
  /// @return Observation vector, or std::nullopt if the measurement should be
  /// rejected.
  [[nodiscard]] virtual std::optional<Eigen::VectorXd>
  Observe(const SensorMeasurement &measurement,
          const UkfUpdateContext &ctx) const = 0;

  /// @brief Returns measurement noise covariance R.
  [[nodiscard]] virtual Eigen::MatrixXd
  NoiseCovariance(const SensorMeasurement &measurement,
                  const UkfUpdateContext &ctx) const = 0;

  /// @brief Computes innovation z_obs - z_pred; override for angle wrapping.
  [[nodiscard]] virtual Eigen::VectorXd
  Innovation(const Eigen::VectorXd &z_obs,
             const Eigen::VectorXd &z_pred) const {
    return z_obs - z_pred;
  }
};

} // namespace falconguide::estimation::ukf
