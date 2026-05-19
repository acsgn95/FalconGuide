#pragma once

#include "falconguide/estimation/backends/ukf/measurement_models/ukf_measurement_model.hpp"

#include <Eigen/Core>
#include <optional>

namespace falconguide::estimation::ukf {

struct UkfMagnetometerOptions {
  Eigen::Vector3d reference_field_enu_tesla{0.0, 2.0e-5, -4.3e-5};
  std::optional<double> sigma_tesla;
  double innovation_gate{0.0};
};

class UkfMagnetometer : public IUkfMeasurementModel {
 public:
  explicit UkfMagnetometer(UkfMagnetometerOptions options = {});

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
  UkfMagnetometerOptions options_;
};

}  // namespace falconguide::estimation::ukf
