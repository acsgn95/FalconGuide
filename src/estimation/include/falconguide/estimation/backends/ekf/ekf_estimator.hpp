#pragma once

#include "falconguide/core/buffers/time_ordered_buffer.hpp"
#include "falconguide/core/coordinates.hpp"
#include "falconguide/core/navigation_state.hpp"
#include "falconguide/core/sensors/gnss.hpp"
#include "falconguide/core/sensors/imu.hpp"
#include "falconguide/estimation/backends/ekf/ekf_state.hpp"
#include "falconguide/estimation/estimator_interface.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace falconguide::estimation::ekf {

struct EkfOptions {
  EstimatorOptions base;
  ImuNoiseModel imu_noise;

  // GNSS solution noise override — if std::nullopt the covariance from
  // GnssSolution itself is used.
  std::optional<double> gnss_position_sigma_m;

  // Minimum seconds between two IMU measurements before propagation is skipped
  // (guards against duplicate or reverse-time packets).
  double min_imu_dt_s{1e-6};

  // Maximum IMU propagation step (splits large gaps to limit linearisation error).
  double max_imu_dt_s{0.05};

  // After this many seconds without any aiding the estimator enters DeadReckoning.
  double dead_reckoning_threshold_s{5.0};
};

// ── EkfEstimator ─────────────────────────────────────────────────────────────
//
// Error-state Extended Kalman Filter for INS/GNSS fusion.
// Implements INavigationEstimator so it is fully interchangeable with
// CeresSlidingWindow or GtsamFactorGraph backends.
//
// Thread-safety: AddMeasurement / ProcessUntil / LatestState may be called
// from different threads; an internal mutex serialises access.
//
class EkfEstimator : public INavigationEstimator {
 public:
  explicit EkfEstimator(EkfOptions options = {});
  ~EkfEstimator() override = default;

  // INavigationEstimator ──────────────────────────────────────────────────────
  [[nodiscard]] EstimatorInfo       Info()    const override;
  [[nodiscard]] const EstimatorOptions& Options() const override;

  EstimatorUpdateResult AddMeasurement(const SensorMeasurement& measurement) override;
  EstimatorUpdateResult ProcessUntil(const core::Timestamp& timestamp)       override;
  void                  Reset()                                               override;

  [[nodiscard]] bool IsInitialized() const override;
  [[nodiscard]] std::optional<core::NavigationState> LatestState() const override;

 private:
  // ── Propagation ────────────────────────────────────────────────────────────
  void PropagateImu(const core::ImuMeasurement& imu);

  // ── Updates ────────────────────────────────────────────────────────────────
  EstimatorUpdateResult UpdateGnss(const core::GnssSolution& gnss);

  // ── Initialisation ─────────────────────────────────────────────────────────
  bool TryInitialiseFromGnss(const core::GnssSolution& gnss);

  // ── Helpers ────────────────────────────────────────────────────────────────
  // Build F (state transition) and G (noise input) matrices for IMU propagation.
  static void BuildFG(const NominalState& nominal,
                      const core::ImuMeasurement& imu,
                      double dt_s,
                      Eigen::Matrix<double, kStateSize, kStateSize>& F,
                      Eigen::Matrix<double, kStateSize, 12>& G);

  // Assemble process noise Q from the IMU noise model and integration step.
  static Eigen::Matrix<double, kStateSize, kStateSize> BuildProcessNoise(
      const ImuNoiseModel& noise, double dt_s);

  // Apply a correction δx to the nominal state and reset the error state to 0.
  void InjectErrorAndReset(const ErrorState& delta_x);

  // Convert internal state to the public NavigationState format.
  [[nodiscard]] core::NavigationState BuildNavigationState() const;

  // ── Data ───────────────────────────────────────────────────────────────────
  EkfOptions options_;
  mutable std::mutex mutex_;

  bool initialised_{false};
  EkfState state_;

  // Buffered IMU measurements waiting to be propagated.
  core::TimeOrderedBuffer<core::ImuMeasurement> imu_buffer_;

  // Last processed IMU timestamp (for dt computation).
  std::optional<core::Timestamp> last_imu_timestamp_;

  // Last aiding timestamp (for dead-reckoning detection).
  std::optional<core::Timestamp> last_aiding_timestamp_;

  // ENU origin, set at first GNSS fix.
  std::optional<core::LocalTangentPlane> local_tangent_plane_;
};

}  // namespace falconguide::estimation::ekf
