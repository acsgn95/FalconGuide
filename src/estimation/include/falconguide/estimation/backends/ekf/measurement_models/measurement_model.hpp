#pragma once

#include "falconguide/core/coordinates.hpp"
#include "falconguide/estimation/backends/ekf/ekf_state.hpp"
#include "falconguide/estimation/estimator_interface.hpp"

#include <Eigen/Core>

namespace falconguide::estimation::ekf {

// ── UpdateContext ─────────────────────────────────────────────────────────────
//
// Contextual information passed to every measurement model at update time.
// Models that do not need a particular field may ignore it.  Adding new fields
// here extends all models without changing their interface signatures.
//
struct UpdateContext {
  // ENU local tangent plane anchored at the first GNSS fix.
  // Null until the estimator has been initialised from a GNSS solution.
  const core::LocalTangentPlane* ltp{nullptr};
};

// ── IMeasurementModel ─────────────────────────────────────────────────────────
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
// this interface, register an instance with EkfEstimator::RegisterMeasurementModel,
// and nothing else needs to change.
//
class IMeasurementModel {
 public:
  virtual ~IMeasurementModel() = default;

  // Human-readable name used in MeasurementUpdateReport.
  [[nodiscard]] virtual std::string_view Name() const { return "Unknown"; }

  // Returns true if this model can process the given measurement variant.
  [[nodiscard]] virtual bool CanHandle(const SensorMeasurement& measurement) const = 0;

  // Performs the EKF measurement update in-place.
  // On success returns Accepted; the engine injects the error state afterwards.
  // Returns Rejected if the innovation gate fails or the measurement is unusable.
  // Returns NotInitialized if required context (e.g. ltp) is not yet available.
  virtual EstimatorUpdateResult Apply(
      NominalState& nominal,
      Eigen::VectorXd& error_state,
      Eigen::MatrixXd& covariance,
      const StateLayout& layout,
      const SensorMeasurement& measurement,
      const UpdateContext& context) = 0;
};

}  // namespace falconguide::estimation::ekf
