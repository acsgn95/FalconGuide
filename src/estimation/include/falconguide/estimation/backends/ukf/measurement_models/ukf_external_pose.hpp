#pragma once

/**
 * @file ukf_external_pose.hpp
 * @brief UKF external pose measurement model.
 */

#include "falconguide/estimation/backends/ukf/measurement_models/ukf_measurement_model.hpp"

#include <optional>

namespace falconguide::estimation::ukf {

/// @brief Options for UKF external-pose fusion.
struct UkfExternalPoseOptions {
  bool use_position{true};
  bool use_orientation{true};
  std::optional<double> position_sigma_m;
  std::optional<double> orientation_sigma_rad;
  double innovation_gate{0.0};
};

/// @brief UKF measurement model for external position and attitude updates.
class UkfExternalPose : public IUkfMeasurementModel {
public:
  explicit UkfExternalPose(UkfExternalPoseOptions options = {});

  [[nodiscard]] bool
  CanHandle(const SensorMeasurement &measurement) const override;
  [[nodiscard]] int
  MeasurementDim(const SensorMeasurement &measurement) const override;

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
  UkfExternalPoseOptions options_;
};

} // namespace falconguide::estimation::ukf
