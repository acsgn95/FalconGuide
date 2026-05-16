#pragma once

#include "falconguide/core/buffers/time_ordered_buffer.hpp"
#include "falconguide/core/coordinates.hpp"
#include "falconguide/estimation/backends/ukf/ukf_state.hpp"
#include "falconguide/estimation/estimator_interface.hpp"

#include <optional>
#include <string>

namespace falconguide::estimation::ukf {

struct UkfOptions {
  EstimatorOptions base;

  MerweSigmaParams sigma_params;
  ImuNoiseModel    imu_noise;

  // Override GNSS measurement noise (m). std::nullopt → use GnssSolution covariance.
  std::optional<double> gnss_position_sigma_m;

  double min_imu_dt_s{1e-6};
  double max_imu_dt_s{0.05};
  double dead_reckoning_threshold_s{5.0};
};

// ── UkfEstimator ──────────────────────────────────────────────────────────────
//
// Unscented Kalman Filter for INS/GNSS fusion.
//
// Advantages over EKF:
//   • Captures second-order nonlinear effects in propagation and update.
//   • No Jacobian derivation required — easier to extend with new measurement
//     types (barometer, magnetometer, etc.).
//   • Better attitude estimation during high-dynamics manoeuvres.
//
// Disadvantages vs EKF:
//   • ~(2n+1) process model evaluations per step (n=15 → 31 evaluations).
//     Roughly 2–3× more expensive than a single EKF propagation step.
//   • Sigma point collapse can occur if covariance becomes ill-conditioned
//     (requires Cholesky regularisation).
//
// Backend: EstimatorBackend::Custom  (identifier "UKF" in EstimatorInfo::name)
//
class UkfEstimator : public INavigationEstimator {
 public:
  explicit UkfEstimator(UkfOptions options = {});
  ~UkfEstimator() override = default;

  [[nodiscard]] EstimatorInfo           Info()    const override;
  [[nodiscard]] const EstimatorOptions& Options() const override;

  EstimatorUpdateResult AddMeasurement(const SensorMeasurement& measurement) override;
  EstimatorUpdateResult ProcessUntil(const core::Timestamp& timestamp)       override;
  void                  Reset()                                               override;

  [[nodiscard]] bool IsInitialized() const override;
  [[nodiscard]] std::optional<core::NavigationState> LatestState() const override;

 private:
  // ── Sigma point management ─────────────────────────────────────────────────
  void GenerateSigmaPoints();

  // ── Propagation ────────────────────────────────────────────────────────────
  // Propagate all sigma points through the nonlinear process model.
  void PropagateImu(const core::ImuMeasurement& imu);

  // Recover mean and covariance from propagated sigma points (unscented transform).
  void RecoverMeanAndCovariance();

  // ── Updates ────────────────────────────────────────────────────────────────
  EstimatorUpdateResult UpdateGnss(const core::GnssSolution& gnss);

  // ── Initialisation ─────────────────────────────────────────────────────────
  bool TryInitialiseFromGnss(const core::GnssSolution& gnss);

  // ── State → NavigationState ────────────────────────────────────────────────
  [[nodiscard]] core::NavigationState BuildNavigationState() const;

  // ── Data ───────────────────────────────────────────────────────────────────
  UkfOptions   options_;
  SigmaWeights weights_;
  UkfState     state_;

  bool initialised_{false};

  core::TimeOrderedBuffer<core::ImuMeasurement> imu_buffer_;
  std::optional<core::Timestamp>                last_imu_timestamp_;
  std::optional<core::Timestamp>                last_aiding_timestamp_;
  std::optional<core::LocalTangentPlane>        local_tangent_plane_;
};

}  // namespace falconguide::estimation::ukf
