#pragma once

/**
 * @file ukf_dvl.hpp
 * @brief UKF Doppler Velocity Log measurement model.
 */

#include "falconguide/estimation/backends/ukf/measurement_models/ukf_measurement_model.hpp"

#include <Eigen/Core>
#include <optional>

namespace falconguide::estimation::ukf {

/// @brief Options for UKF DVL fusion.
struct UkfDvlOptions {
  Eigen::Matrix3d dvl_to_body_rotation{Eigen::Matrix3d::Identity()};
  bool bottom_track_only{false};
  std::optional<double> sigma_mps;
  double innovation_gate{0.0};
};

/// @brief UKF measurement model for 3-axis DVL velocity updates.
class UkfDvl : public IUkfMeasurementModel {
public:
  explicit UkfDvl(UkfDvlOptions options = {});

  [[nodiscard]] bool
  CanHandle(const SensorMeasurement &measurement) const override;
  [[nodiscard]] int MeasurementDim(const SensorMeasurement &) const override {
    return 3;
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
  UkfDvlOptions options_;
};

} // namespace falconguide::estimation::ukf
