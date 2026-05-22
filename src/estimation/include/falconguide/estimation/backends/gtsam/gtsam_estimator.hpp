#pragma once

/**
 * @file gtsam_estimator.hpp
 * @brief GTSAM factor-graph smoother backend.
 */

#include "falconguide/estimation/backends/gtsam/gtsam_state.hpp"
#include "falconguide/estimation/estimator_interface.hpp"

#include <optional>
#include <string>

namespace falconguide::estimation::gtsam_backend {

/// @brief Options for the GTSAM factor-graph estimator.
struct GtsamOptions {
  EstimatorOptions base; ///< Backend-independent options.

  SmootherMode smoother_mode{
      SmootherMode::ISAM2}; ///< Smoother implementation mode.
  ISAM2Policy isam2_policy; ///< iSAM2 relinearization policy.

  // Fixed-lag smoother lag window.
  double fixed_lag_s{5.0}; ///< Fixed-lag smoother window in seconds.

  // New node is created if moved at least this far since last node.
  double node_min_distance_m{
      0.5}; ///< Create a new node after this translation.
  double node_min_rotation_rad{0.1}; ///< Create a new node after this rotation.

  // Noise sigmas for GNSS position factor (m). If std::nullopt, uses
  // GnssSolution covariance.
  std::optional<double>
      gnss_position_sigma_m; ///< Optional GNSS position-noise override.

  // Noise sigmas for vision factors (pixels). Only active when visual factors
  // enabled.
  double vision_reprojection_sigma_px{
      1.5}; ///< Vision reprojection sigma in pixels.

  bool enable_visual_factors{false}; ///< Enables visual reprojection factors.
  bool enable_gnss_velocity_factor{true}; ///< Enables GNSS velocity factors.
};

// ── GtsamFactorGraphEstimator
// ─────────────────────────────────────────────────
//
// Incremental factor graph smoother backed by GTSAM's iSAM2 or fixed-lag
// smoother. Supports tightly-integrated GNSS, IMU, and (optionally) visual
// reprojection factors.
//
// Key properties vs EKF:
//   • Full nonlinear re-linearisation — better accuracy on aggressive
//   manoeuvres. • iSAM2 amortises cost: only variables affected by new factors
//   are updated. • Fixed-lag mode bounds memory to a configurable time horizon.
//
// Backend: EstimatorBackend::GtsamFactorGraph
//
class GtsamFactorGraphEstimator : public INavigationEstimator {
public:
  /// @brief Constructs a GTSAM factor-graph estimator.
  explicit GtsamFactorGraphEstimator(GtsamOptions options = {});
  /// @brief Virtual destructor for backend polymorphism.
  ~GtsamFactorGraphEstimator() override = default;

  [[nodiscard]] EstimatorInfo Info() const override;
  [[nodiscard]] const EstimatorOptions &Options() const override;

  MeasurementUpdateReport
  AddMeasurement(const SensorMeasurement &measurement) override;
  EstimatorUpdateResult ProcessUntil(const core::Timestamp &timestamp) override;
  void Reset() override;

  [[nodiscard]] bool IsInitialized() const override;
  [[nodiscard]] std::optional<core::NavigationState>
  LatestState() const override;

  /// @brief Exposes the factor graph state for diagnostics and offline
  /// analysis.
  [[nodiscard]] const FactorGraphState &GraphState() const;

private:
  GtsamOptions options_;
  FactorGraphState graph_;
  bool initialised_{false};
};

} // namespace falconguide::estimation::gtsam_backend
