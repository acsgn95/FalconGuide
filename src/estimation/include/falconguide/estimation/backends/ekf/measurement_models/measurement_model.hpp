#pragma once

/**
 * @file measurement_model.hpp
 * @brief Plugin interface for EKF measurement update models.
 */

#include "falconguide/core/coordinates.hpp"
#include "falconguide/estimation/backends/ekf/ekf_state.hpp"
#include "falconguide/estimation/estimator_interface.hpp"

#include <Eigen/Core>

namespace falconguide::estimation::ekf {

// ── UpdateContext
// ─────────────────────────────────────────────────────────────
//
// Contextual information passed to every measurement model at update time.
// Models that do not need a particular field may ignore it.  Adding new fields
// here extends all models without changing their interface signatures.
//
struct UpdateContext {
  // ENU local tangent plane anchored at the first GNSS fix.
  // Null until the estimator has been initialised from a GNSS solution.
  const core::LocalTangentPlane *ltp{
      nullptr}; ///< Local tangent plane, or nullptr before initialization.
};

// ── IMeasurementModel
// ─────────────────────────────────────────────────────────
//
// Plugin interface for EKF measurement update steps.
//
// Each concrete model handles one or more SensorMeasurement variants and
// performs the standard EKF update in-place:
//   K = P H' (H P H' + R)^{-1}
//   δx += K (z - h(x))
//   P = (I − K H) P (I − K H)' + K R K'   [Joseph form]
//
// After Apply() returns Accepted, the engine calls InjectErrorAndReset() to
// fold δx into the nominal state and reset the error vector to zero.
//
// To add support for a new sensor (barometer, star tracker, DVL, …) implement
// this interface, register an instance with
// EkfEstimator::RegisterMeasurementModel, and nothing else needs to change.
//
class IMeasurementModel {
public:
  /// @brief Virtual destructor for interface use.
  virtual ~IMeasurementModel() = default;

  /// @brief Human-readable name used in MeasurementUpdateReport.
  [[nodiscard]] virtual std::string_view Name() const { return "Unknown"; }

  /// @brief Returns true if this model can process the given measurement
  /// variant.
  [[nodiscard]] virtual bool
  CanHandle(const SensorMeasurement &measurement) const = 0;

  /// @brief Performs the EKF measurement update in-place.
  /// @return Accepted on success, Rejected on gating failure, or NotInitialized
  /// for missing context.
  virtual EstimatorUpdateResult
  Apply(NominalState &nominal, Eigen::VectorXd &error_state,
        Eigen::MatrixXd &covariance, const StateLayout &layout,
        const SensorMeasurement &measurement, const UpdateContext &context) = 0;
};

} // namespace falconguide::estimation::ekf
