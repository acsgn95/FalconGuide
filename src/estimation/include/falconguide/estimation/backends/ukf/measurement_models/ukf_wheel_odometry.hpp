#pragma once

#include "falconguide/estimation/backends/ukf/measurement_models/ukf_measurement_model.hpp"

namespace falconguide::estimation::ukf {

struct UkfWheelOdometryOptions {
  double sigma_linear_mps{0.1};
  double innovation_gate{0.0};
};

class UkfWheelOdometry : public IUkfMeasurementModel {
 public:
  explicit UkfWheelOdometry(UkfWheelOdometryOptions options = {});

  [[nodiscard]] bool CanHandle(const SensorMeasurement& measurement) const override;
  [[nodiscard]] int  MeasurementDim(const SensorMeasurement&) const override { return 3; }

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
  UkfWheelOdometryOptions options_;
};

}  // namespace falconguide::estimation::ukf
