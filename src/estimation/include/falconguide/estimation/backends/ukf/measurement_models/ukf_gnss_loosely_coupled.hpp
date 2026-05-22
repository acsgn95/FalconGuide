#pragma once

/**
 * @file ukf_gnss_loosely_coupled.hpp
 * @brief UKF loosely-coupled GNSS solution measurement model.
 */

#include "falconguide/estimation/backends/ukf/measurement_models/ukf_measurement_model.hpp"

#include <optional>

namespace falconguide::estimation::ukf {

/// @brief Options for UKF loosely-coupled GNSS fusion.
struct UkfGnssLooselyCoupledOptions {
    std::optional<double> position_sigma_m;
    std::optional<double> velocity_sigma_mps;
};

// ── UkfGnssLooselyCoupled
// ─────────────────────────────────────────────────────
//
// UKF measurement model for loosely coupled GNSS fusion (GnssSolution).
//
// h(xᵢ) = [pᵢ_enu ; vᵢ_enu]   (6-dimensional, no linearisation needed)
//
// This is markedly simpler than the EKF equivalent: no H Jacobian, no
// coordinate-frame rotation inside the observation equation — the sigma
// points are already in ENU, so Predict() just returns [p ; v].
//
/// @brief UKF measurement model for receiver-computed GNSS position and
/// velocity.
class UkfGnssLooselyCoupled : public IUkfMeasurementModel {
   public:
    explicit UkfGnssLooselyCoupled(UkfGnssLooselyCoupledOptions options = {});

    [[nodiscard]] bool CanHandle(const SensorMeasurement &measurement) const override;
    [[nodiscard]] int MeasurementDim(const SensorMeasurement &measurement) const override;

    [[nodiscard]] Eigen::VectorXd Predict(const NominalState &sigma_state, const UkfUpdateContext &ctx) const override;

    [[nodiscard]] std::optional<Eigen::VectorXd> Observe(const SensorMeasurement &measurement,
                                                         const UkfUpdateContext &ctx) const override;

    [[nodiscard]] Eigen::MatrixXd NoiseCovariance(const SensorMeasurement &measurement,
                                                  const UkfUpdateContext &ctx) const override;

   private:
    UkfGnssLooselyCoupledOptions options_;
};

}  // namespace falconguide::estimation::ukf
