#pragma once

#include "falconguide/estimation/backends/gtsam/gtsam_state.hpp"
#include "falconguide/estimation/estimator_interface.hpp"

#include <optional>
#include <string>

namespace falconguide::estimation::gtsam_backend {

struct GtsamOptions {
  EstimatorOptions base;

  SmootherMode smoother_mode{SmootherMode::ISAM2};
  ISAM2Policy  isam2_policy;

  // Fixed-lag smoother lag window.
  double fixed_lag_s{5.0};

  // New node is created if moved at least this far since last node.
  double node_min_distance_m{0.5};
  double node_min_rotation_rad{0.1};

  // Noise sigmas for GNSS position factor (m). If std::nullopt, uses GnssSolution covariance.
  std::optional<double> gnss_position_sigma_m;

  // Noise sigmas for vision factors (pixels). Only active when visual factors enabled.
  double vision_reprojection_sigma_px{1.5};

  bool enable_visual_factors{false};
  bool enable_gnss_velocity_factor{true};
};

// ── GtsamFactorGraphEstimator ─────────────────────────────────────────────────
//
// Incremental factor graph smoother backed by GTSAM's iSAM2 or fixed-lag
// smoother. Supports tightly-integrated GNSS, IMU, and (optionally) visual
// reprojection factors.
//
// Key properties vs EKF:
//   • Full nonlinear re-linearisation — better accuracy on aggressive manoeuvres.
//   • iSAM2 amortises cost: only variables affected by new factors are updated.
//   • Fixed-lag mode bounds memory to a configurable time horizon.
//
// Backend: EstimatorBackend::GtsamFactorGraph
//
class GtsamFactorGraphEstimator : public INavigationEstimator {
 public:
  explicit GtsamFactorGraphEstimator(GtsamOptions options = {});
  ~GtsamFactorGraphEstimator() override = default;

  [[nodiscard]] EstimatorInfo           Info()    const override;
  [[nodiscard]] const EstimatorOptions& Options() const override;

  MeasurementUpdateReport AddMeasurement(const SensorMeasurement& measurement) override;
  EstimatorUpdateResult ProcessUntil(const core::Timestamp& timestamp)       override;
  void                  Reset()                                               override;

  [[nodiscard]] bool IsInitialized() const override;
  [[nodiscard]] std::optional<core::NavigationState> LatestState() const override;

  // Expose the full factor graph state for diagnostics / offline analysis.
  [[nodiscard]] const FactorGraphState& GraphState() const;

 private:
  GtsamOptions      options_;
  FactorGraphState  graph_;
  bool              initialised_{false};
};

}  // namespace falconguide::estimation::gtsam_backend
