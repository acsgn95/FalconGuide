#pragma once

#include "falconguide/estimation/backends/ceres/ceres_state.hpp"
#include "falconguide/estimation/estimator_interface.hpp"

#include <cstddef>
#include <optional>
#include <string>

namespace falconguide::estimation::ceres_backend {

// Loss function type for robustifying residuals against outliers.
enum class LossFunctionType {
  None,     // least squares (no robustification)
  Huber,    // Huber loss — good default for GNSS and vision
  Cauchy,   // heavier tail than Huber — useful for image feature outliers
  Tukey,    // hard rejection above threshold
};

// Solver convergence criterion.
enum class SolverStrategy {
  FastOneIteration,   // single Gauss-Newton step (low latency, low accuracy)
  IteratedFull,       // full Levenberg-Marquardt until convergence
  AdaptiveBudget,     // run as many iterations as time budget allows
};

struct CeresOptions {
  EstimatorOptions base;

  // Window parameters.
  std::size_t max_keyframes{10};
  double keyframe_min_distance_m{0.5};     // new KF if moved this far
  double keyframe_min_rotation_rad{0.1};   // or rotated this much

  // Solver parameters.
  SolverStrategy solver_strategy{SolverStrategy::IteratedFull};
  int max_solver_iterations{10};
  double solver_time_budget_s{0.05};       // used with AdaptiveBudget

  // Robustification.
  LossFunctionType gnss_loss{LossFunctionType::Huber};
  LossFunctionType vision_loss{LossFunctionType::Huber};
  double huber_loss_parameter{1.0};

  // Whether to include visual reprojection factors (requires camera frames).
  bool enable_visual_factors{false};
};

// ── CeresSlidingWindowEstimator ───────────────────────────────────────────────
//
// Nonlinear sliding-window smoother implemented with Ceres Solver.
// Maintains a fixed-size window of keyframes and marginalises old states
// using a Schur-complement prior to bound computational cost.
//
// Backend: EstimatorBackend::CeresSlidingWindow
//
class CeresSlidingWindowEstimator : public INavigationEstimator {
 public:
  explicit CeresSlidingWindowEstimator(CeresOptions options = {});
  ~CeresSlidingWindowEstimator() override = default;

  [[nodiscard]] EstimatorInfo           Info()    const override;
  [[nodiscard]] const EstimatorOptions& Options() const override;

  EstimatorUpdateResult AddMeasurement(const SensorMeasurement& measurement) override;
  EstimatorUpdateResult ProcessUntil(const core::Timestamp& timestamp)       override;
  void                  Reset()                                               override;

  [[nodiscard]] bool IsInitialized() const override;
  [[nodiscard]] std::optional<core::NavigationState> LatestState() const override;

  // Expose sliding window for diagnostics / visualisation.
  [[nodiscard]] const SlidingWindowState& WindowState() const;

 private:
  CeresOptions options_;
  SlidingWindowState window_;
  bool initialised_{false};
};

}  // namespace falconguide::estimation::ceres_backend
