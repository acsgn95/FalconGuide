#pragma once

#include "falconguide/estimation/backends/ukf/measurement_models/ukf_measurement_model.hpp"

#include <optional>

namespace falconguide::estimation::ukf {

struct UkfOpticalFlowOptions {
  double fallback_altitude_m{10.0};
  double sigma_radps{0.01};
  double innovation_gate{0.0};
};

class UkfOpticalFlow : public IUkfMeasurementModel {
 public:
  explicit UkfOpticalFlow(UkfOpticalFlowOptions options = {});

  [[nodiscard]] bool CanHandle(const SensorMeasurement& measurement) const override;
  [[nodiscard]] int  MeasurementDim(const SensorMeasurement&) const override { return 2; }

  [[nodiscard]] Eigen::VectorXd Predict(
      const NominalState& sigma_state,
      const UkfUpdateContext& ctx) const override;

  [[nodiscard]] std::optional<Eigen::VectorXd> Observe(
      const SensorMeasurement& measurement,
      const UkfUpdateContext& ctx) const override;

  [[nodiscard]] Eigen::MatrixXd NoiseCovariance(
      const SensorMeasurement& measurement,
      const UkfUpdateContext& ctx) const override;

 private:
  UkfOpticalFlowOptions options_;
};

}  // namespace falconguide::estimation::ukf
