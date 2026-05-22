#pragma once

/**
 * @file ukf_airspeed.hpp
 * @brief UKF scalar airspeed measurement model.
 */

#include "falconguide/estimation/backends/ukf/measurement_models/ukf_measurement_model.hpp"

#include <Eigen/Core>

namespace falconguide::estimation::ukf {

/// @brief Options for UKF airspeed fusion.
struct UkfAirspeedOptions {
  Eigen::Vector3d wind_enu_mps{0.0, 0.0, 0.0};
  double sigma_mps{0.5};
  double innovation_gate{0.0};
};

/// @brief UKF measurement model for airspeed magnitude updates.
class UkfAirspeed : public IUkfMeasurementModel {
public:
  explicit UkfAirspeed(UkfAirspeedOptions options = {});

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
  UkfAirspeedOptions options_;
};

} // namespace falconguide::estimation::ukf
