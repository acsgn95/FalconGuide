#pragma once

/**
 * @file ukf_echo_sounder.hpp
 * @brief UKF echo-sounder depth measurement model.
 */

#include "falconguide/estimation/backends/ukf/measurement_models/ukf_measurement_model.hpp"

#include <optional>

namespace falconguide::estimation::ukf {

/// @brief Options for UKF echo-sounder fusion.
struct UkfEchoSounderOptions {
  double sound_speed_mps{1500.0};
  double depth_origin_m{0.0};
  std::optional<double> sigma_m;
  double innovation_gate{0.0};
};

/// @brief UKF measurement model for scalar underwater depth updates.
class UkfEchoSounder : public IUkfMeasurementModel {
public:
  explicit UkfEchoSounder(UkfEchoSounderOptions options = {});

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
  UkfEchoSounderOptions options_;
};

} // namespace falconguide::estimation::ukf
