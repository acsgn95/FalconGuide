#pragma once

/**
 * @file ukf_estimator.hpp
 * @brief Unscented Kalman Filter backend for multi-sensor inertial navigation.
 */

#include "falconguide/core/buffers/time_ordered_buffer.hpp"
#include "falconguide/core/coordinates.hpp"
#include "falconguide/estimation/backends/ukf/measurement_models/ukf_measurement_model.hpp"
#include "falconguide/estimation/backends/ukf/ukf_state.hpp"
#include "falconguide/estimation/estimator_interface.hpp"

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace falconguide::estimation::ukf {

struct UkfOptions {
    EstimatorOptions base;  ///< Backend-independent options.

    MerweSigmaParams sigma_params;  ///< Sigma-point spread and weighting parameters.
    ImuNoiseModel imu_noise;        ///< Continuous-time IMU noise model.

    // Override GNSS measurement noise (m). std::nullopt → use GnssSolution
    // covariance.
    std::optional<double> gnss_position_sigma_m;  ///< Optional GNSS position-noise override.

    double min_imu_dt_s{1e-6};               ///< Minimum accepted IMU sample spacing.
    double max_imu_dt_s{0.05};               ///< Maximum propagation step.
    double dead_reckoning_threshold_s{5.0};  ///< Aiding age that marks the solution as dead reckoning.
};

// ── UkfEstimator
// ──────────────────────────────────────────────────────────────
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
    /// @brief Constructs a UKF estimator from options.
    explicit UkfEstimator(UkfOptions options = {});
    /// @brief Virtual destructor for backend polymorphism.
    ~UkfEstimator() override = default;

    /// @brief Registers a sensor measurement model tried in registration order.
    void RegisterMeasurementModel(std::unique_ptr<IUkfMeasurementModel> model);

    /// @copydoc INavigationEstimator::Info
    [[nodiscard]] EstimatorInfo Info() const override;
    /// @copydoc INavigationEstimator::Options
    [[nodiscard]] const EstimatorOptions &Options() const override;

    /// @copydoc INavigationEstimator::AddMeasurement
    MeasurementUpdateReport AddMeasurement(const SensorMeasurement &measurement) override;
    /// @copydoc INavigationEstimator::ProcessUntil
    EstimatorUpdateResult ProcessUntil(const core::Timestamp &timestamp) override;
    /// @copydoc INavigationEstimator::Reset
    void Reset() override;

    /// @copydoc INavigationEstimator::IsInitialized
    [[nodiscard]] bool IsInitialized() const override;
    /// @copydoc INavigationEstimator::LatestState
    [[nodiscard]] std::optional<core::NavigationState> LatestState() const override;

   private:
    void GenerateSigmaPoints();
    void PropagateImu(const core::ImuMeasurement &imu, double dt_s);
    void PropagateTo(const core::Timestamp &target);
    [[nodiscard]] StateCovariance BuildProcessNoise(const core::ImuMeasurement &imu, double dt_s) const;

    MeasurementUpdateReport ApplyMeasurementModel(IUkfMeasurementModel &model, const SensorMeasurement &measurement,
                                                  const UkfUpdateContext &ctx);

    bool TryInitialiseFromGnss(const core::GnssSolution &gnss);
    [[nodiscard]] core::NavigationState BuildNavigationState() const;

    UkfOptions options_;
    SigmaWeights weights_;
    UkfState state_;
    mutable std::mutex mutex_;

    bool initialised_{false};

    std::vector<std::unique_ptr<IUkfMeasurementModel>> measurement_models_;
    core::TimeOrderedBuffer<core::ImuMeasurement> imu_buffer_;
    std::optional<core::Timestamp> last_imu_timestamp_;
    std::optional<core::Timestamp> last_aiding_timestamp_;
    std::optional<core::LocalTangentPlane> local_tangent_plane_;
};

}  // namespace falconguide::estimation::ukf
