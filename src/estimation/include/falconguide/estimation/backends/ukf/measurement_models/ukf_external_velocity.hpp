#pragma once

/**
 * @file ukf_external_velocity.hpp
 * @brief UKF external velocity measurement model.
 */

#include "falconguide/estimation/backends/ukf/measurement_models/ukf_measurement_model.hpp"

#include <optional>

namespace falconguide::estimation::ukf {

/// @brief Options for UKF external-velocity fusion.
struct UkfExternalVelocityOptions {
    std::optional<double> sigma_linear_mps;
    double innovation_gate{0.0};
};

/// @brief UKF measurement model for external body-frame velocity updates.
class UkfExternalVelocity : public IUkfMeasurementModel {
   public:
    explicit UkfExternalVelocity(UkfExternalVelocityOptions options = {});

    [[nodiscard]] bool CanHandle(const SensorMeasurement &measurement) const override;
    [[nodiscard]] int MeasurementDim(const SensorMeasurement &) const override { return 3; }

    [[nodiscard]] Eigen::VectorXd Predict(const NominalState &sigma_state, const UkfUpdateContext &ctx) const override;

    [[nodiscard]] std::optional<Eigen::VectorXd> Observe(const SensorMeasurement &measurement,
                                                         const UkfUpdateContext &ctx) const override;

    [[nodiscard]] Eigen::MatrixXd NoiseCovariance(const SensorMeasurement &measurement,
                                                  const UkfUpdateContext &ctx) const override;

   private:
    UkfExternalVelocityOptions options_;
};

}  // namespace falconguide::estimation::ukf
