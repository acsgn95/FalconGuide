#pragma once

/**
 * @file ekf_estimator.hpp
 * @brief Error-state Extended Kalman Filter backend.
 */

#include "falconguide/core/buffers/time_ordered_buffer.hpp"
#include "falconguide/core/coordinates.hpp"
#include "falconguide/estimation/backends/ekf/ekf_state.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/measurement_model.hpp"
#include "falconguide/estimation/estimator_interface.hpp"

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace falconguide::estimation::ekf {

// ── EkfOptions
// ────────────────────────────────────────────────────────────────
struct EkfOptions {
  EstimatorOptions base;   ///< Backend-independent options.
  ImuNoiseModel imu_noise; ///< Continuous-time IMU noise model.

  // Guard against duplicate or reverse-time IMU packets.
  double min_imu_dt_s{1e-6}; ///< Minimum accepted IMU sample spacing.

  // Splits large propagation gaps to limit linearisation error.
  double max_imu_dt_s{
      0.05}; ///< Maximum propagation step before splitting or limiting.

  // After this many seconds without any aiding the output status becomes
  // DeadReckoning.
  double dead_reckoning_threshold_s{
      5.0}; ///< Aiding age that marks the solution as dead reckoning.

  // ── Optional state extensions ─────────────────────────────────────────────
  // Set true to add StateSegmentId::GnssClock (required for tightly coupled).
  bool enable_gnss_clock_state{false}; ///< Adds GNSS receiver clock states.

  // Set true to add StateSegmentId::BaroBias.
  bool enable_baro_bias_state{false}; ///< Adds a barometer bias state.
};

// ── EkfEstimator
// ──────────────────────────────────────────────────────────────
//
// Error-state Extended Kalman Filter for multi-sensor inertial navigation.
//
// The state space is built dynamically from EkfOptions at construction time,
// so the layout can be extended without recompiling the filter core.
//
// Sensor fusion is fully plugin-based: register any number of IMeasurementModel
// instances (GNSS loosely coupled, tightly coupled, barometer, magnetometer,
// star tracker, DVL, …).  The engine dispatches each incoming measurement to
// the first model that reports CanHandle() == true.
//
// Thread-safety: AddMeasurement, ProcessUntil, and LatestState may be called
// from different threads; an internal mutex serialises all state access.
//
class EkfEstimator : public INavigationEstimator {
public:
  /// @brief Constructs an EKF estimator from options.
  explicit EkfEstimator(EkfOptions options = {});
  /// @brief Virtual destructor for backend polymorphism.
  ~EkfEstimator() override = default;

  /// @brief Registers a measurement model tried in registration order.
  void RegisterMeasurementModel(std::unique_ptr<IMeasurementModel> model);

  /// @copydoc INavigationEstimator::Info
  [[nodiscard]] EstimatorInfo Info() const override;
  /// @copydoc INavigationEstimator::Options
  [[nodiscard]] const EstimatorOptions &Options() const override;

  // IMU → buffered for propagation.
  // GnssSolution on first call → initialises state and LTP.
  // All other measurement types → dispatched to registered models after
  //   propagating the IMU buffer to the measurement timestamp.
  MeasurementUpdateReport
  AddMeasurement(const SensorMeasurement &measurement) override;

  /// @brief Propagates all buffered IMU up to and including @p timestamp.
  EstimatorUpdateResult ProcessUntil(const core::Timestamp &timestamp) override;

  /// @copydoc INavigationEstimator::Reset
  void Reset() override;

  /// @copydoc INavigationEstimator::IsInitialized
  [[nodiscard]] bool IsInitialized() const override;
  /// @copydoc INavigationEstimator::LatestState
  [[nodiscard]] std::optional<core::NavigationState>
  LatestState() const override;

private:
  // ── Layout ───────────────────────────────────────────────────────────────
  [[nodiscard]] StateLayout BuildLayout() const;

  // ── Propagation ──────────────────────────────────────────────────────────
  // Propagate nominal state and error covariance forward by dt_s seconds
  // using one IMU measurement.
  void PropagateImu(const core::ImuMeasurement &imu, double dt_s);

  // Pop and propagate all buffered IMU with timestamp <= target.
  void PropagateTo(const core::Timestamp &target);

  // ── Update helpers ────────────────────────────────────────────────────────
  // Add δx to the nominal state and zero the error vector.
  void InjectErrorAndReset();

  // ── Initialisation ────────────────────────────────────────────────────────
  bool TryInitialiseFromGnss(const core::GnssSolution &gnss);

  // ── Output ────────────────────────────────────────────────────────────────
  [[nodiscard]] core::NavigationState BuildNavigationState() const;

  // ── Data ─────────────────────────────────────────────────────────────────
  EkfOptions options_;
  mutable std::mutex mutex_;

  bool initialised_{false};
  EkfState state_;

  std::vector<std::unique_ptr<IMeasurementModel>> measurement_models_;

  core::TimeOrderedBuffer<core::ImuMeasurement> imu_buffer_;
  std::optional<core::Timestamp> last_aiding_timestamp_;
  std::optional<core::LocalTangentPlane> local_tangent_plane_;
};

} // namespace falconguide::estimation::ekf
