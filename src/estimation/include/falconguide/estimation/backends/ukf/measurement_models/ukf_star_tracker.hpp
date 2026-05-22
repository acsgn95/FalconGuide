#pragma once

/**
 * @file ukf_star_tracker.hpp
 * @brief UKF star-tracker attitude measurement model.
 */

#include "falconguide/estimation/backends/ukf/measurement_models/ukf_measurement_model.hpp"

#include <Eigen/Geometry>

namespace falconguide::estimation::ukf {

/// @brief Options for UKF star-tracker fusion.
struct UkfStarTrackerOptions {
    Eigen::Quaterniond star_tracker_to_body{Eigen::Quaterniond::Identity()};
    double sigma_yaw_rad{0.001};
    double sigma_pitch_rad{0.001};
    double sigma_roll_rad{0.001};
    bool use_pitch_roll{true};
    double innovation_gate{0.0};
};

// ── UkfStarTracker
// ────────────────────────────────────────────────────────────
//
// Attitude update from a star tracker.
// Predict() returns the rotation vector LogMapSo3(q_body_to_enu) — a 3-vector
// living in the tangent space of SO(3), which the UKF treats as a Euclidean
// measurement for the purpose of computing the weighted mean and covariance.
//
// Innovation override handles angle wrapping.
//
/// @brief UKF measurement model for celestial attitude updates.
class UkfStarTracker : public IUkfMeasurementModel {
   public:
    explicit UkfStarTracker(UkfStarTrackerOptions options = {});

    [[nodiscard]] bool CanHandle(const SensorMeasurement &measurement) const override;
    [[nodiscard]] int MeasurementDim(const SensorMeasurement &m) const override;

    [[nodiscard]] Eigen::VectorXd Predict(const NominalState &sigma_state, const UkfUpdateContext &ctx) const override;

    [[nodiscard]] std::optional<Eigen::VectorXd> Observe(const SensorMeasurement &measurement,
                                                         const UkfUpdateContext &ctx) const override;

    [[nodiscard]] Eigen::MatrixXd NoiseCovariance(const SensorMeasurement &measurement,
                                                  const UkfUpdateContext &ctx) const override;

    [[nodiscard]] Eigen::VectorXd Innovation(const Eigen::VectorXd &z_obs,
                                             const Eigen::VectorXd &z_pred) const override;

   private:
    UkfStarTrackerOptions options_;
};

}  // namespace falconguide::estimation::ukf
