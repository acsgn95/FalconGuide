#pragma once

/**
 * @file ukf_barometer.hpp
 * @brief UKF altitude update from barometer pressure or altitude.
 */

#include "falconguide/estimation/backends/ukf/measurement_models/ukf_measurement_model.hpp"

#include <optional>

namespace falconguide::estimation::ukf {

/// @brief Options for UKF barometer fusion.
struct UkfBarometerOptions {
  std::optional<double> altitude_sigma_m;
  double isa_sea_level_pressure_pa{101325.0};
  double innovation_gate{0.0};
};

/// @brief UKF measurement model for scalar altitude updates.
class UkfBarometer : public IUkfMeasurementModel {
public:
  explicit UkfBarometer(UkfBarometerOptions options = {});

  [[nodiscard]] bool
  CanHandle(const SensorMeasurement &measurement) const override;
  [[nodiscard]] int MeasurementDim(const SensorMeasurement &) const override {
    return 1;
  }

  [[nodiscard]] Eigen::VectorXd
  Predict(const NominalState &sigma_state,
          const UkfUpdateContext &ctx) const override;

  [[nodiscard]] std::optional<Eigen::VectorXd>
  Observe(const SensorMeasurement &measurement,
          const UkfUpdateContext &ctx) const override;

  [[nodiscard]] Eigen::MatrixXd
  NoiseCovariance(const SensorMeasurement &measurement,
                  const UkfUpdateContext &ctx) const override;

private:
  UkfBarometerOptions options_;
};

} // namespace falconguide::estimation::ukf
