#pragma once

/**
 * @file ceres_estimator.hpp
 * @brief Ceres sliding-window nonlinear smoother backend.
 */

#include "falconguide/estimation/backends/ceres/ceres_state.hpp"
#include "falconguide/estimation/estimator_interface.hpp"

#include <cstddef>
#include <memory>
#include <optional>
#include <string>

namespace falconguide::estimation::ceres_backend {

// Loss function type for robustifying residuals against outliers.
enum class LossFunctionType {
    None,    ///< Least squares without robustification.
    Huber,   ///< Huber robust loss; good default for GNSS and vision.
    Cauchy,  ///< Heavier-tailed robust loss for outlier-prone features.
    Tukey,   ///< Tukey loss with hard rejection above threshold.
};

// Solver convergence criterion.
enum class SolverStrategy {
    FastOneIteration,  ///< Single low-latency Gauss-Newton step.
    IteratedFull,      ///< Full Levenberg-Marquardt solve until convergence.
    AdaptiveBudget,    ///< Runs as many iterations as the time budget allows.
};

/// @brief Options for the Ceres sliding-window estimator.
struct CeresOptions {
    EstimatorOptions base;  ///< Backend-independent options.

    // Window parameters.
    std::size_t max_keyframes{10};          ///< Maximum keyframes retained in the window.
    double keyframe_min_distance_m{0.5};    ///< Create a keyframe after this translation.
    double keyframe_min_rotation_rad{0.1};  ///< Create a keyframe after this rotation.

    // Solver parameters.
    SolverStrategy solver_strategy{SolverStrategy::IteratedFull};  ///< Solver iteration policy.
    int max_solver_iterations{10};                                 ///< Hard solver iteration cap.
    double solver_time_budget_s{0.05};                             ///< Time budget used with AdaptiveBudget.

    // Robustification.
    LossFunctionType gnss_loss{LossFunctionType::Huber};    ///< Robust loss for GNSS factors.
    LossFunctionType vision_loss{LossFunctionType::Huber};  ///< Robust loss for vision factors.
    double huber_loss_parameter{1.0};                       ///< Huber loss parameter.

    // Whether to include visual reprojection factors (requires camera frames).
    bool enable_visual_factors{false};  ///< Enables visual reprojection factors.
};

// ── CeresSlidingWindowEstimator
// ───────────────────────────────────────────────
//
// Nonlinear sliding-window smoother implemented with Ceres Solver.
// Maintains a fixed-size window of keyframes and marginalises old states
// using a Schur-complement prior to bound computational cost.
//
// Backend: EstimatorBackend::CeresSlidingWindow
//
class CeresSlidingWindowEstimator : public INavigationEstimator {
   public:
    /// @brief Constructs a Ceres sliding-window estimator.
    explicit CeresSlidingWindowEstimator(CeresOptions options = {});
    /// @brief Virtual destructor for backend polymorphism.
    ~CeresSlidingWindowEstimator() override;

    [[nodiscard]] EstimatorInfo Info() const override;
    [[nodiscard]] const EstimatorOptions &Options() const override;

    MeasurementUpdateReport AddMeasurement(const SensorMeasurement &measurement) override;
    EstimatorUpdateResult ProcessUntil(const core::Timestamp &timestamp) override;
    void Reset() override;

    [[nodiscard]] bool IsInitialized() const override;
    [[nodiscard]] std::optional<core::NavigationState> LatestState() const override;

    /// @brief Exposes sliding window state for diagnostics and visualization.
    [[nodiscard]] const SlidingWindowState &WindowState() const;

   private:
    class Impl;

    CeresOptions options_;
    std::unique_ptr<Impl> impl_;
};

}  // namespace falconguide::estimation::ceres_backend
