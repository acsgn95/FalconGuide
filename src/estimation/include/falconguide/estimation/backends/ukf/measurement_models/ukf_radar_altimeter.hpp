#pragma once

#include "falconguide/estimation/backends/ukf/measurement_models/ukf_measurement_model.hpp"

#include <optional>

namespace falconguide::estimation::ukf {

struct UkfRadarAltimeterOptions {
  double terrain_elevation_m{0.0};
  std::optional<double> sigma_m;
  double innovation_gate{0.0};
};

class UkfRadarAltimeter : public IUkfMeasurementModel {
 public:
  explicit UkfRadarAltimeter(UkfRadarAltimeterOptions options = {});

  [[nodiscard]] bool CanHandle(const SensorMeasurement& measurement) const override;
  [[nodiscard]] int  MeasurementDim(const SensorMeasurement&) const override { return 1; }

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
  UkfRadarAltimeterOptions options_;
};

}  // namespace falconguide::estimation::ukf
