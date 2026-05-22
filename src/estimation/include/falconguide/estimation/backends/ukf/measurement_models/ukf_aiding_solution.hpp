#pragma once

#include "falconguide/estimation/backends/ukf/measurement_models/ukf_measurement_model.hpp"

#include <optional>

namespace falconguide::estimation::ukf {

struct UkfAidingSolutionOptions {
  double min_match_score{0.0};
  std::optional<double> position_sigma_m;
  std::optional<double> velocity_sigma_mps;
  std::optional<double> attitude_sigma_rad;
  double innovation_gate{0.0};
};

// ── UkfAidingSolution ─────────────────────────────────────────────────────────
//
// Handles AidingSolution (VO, TERCOM, GeoRef, SLAM, …).
// MeasurementDim is dynamic: 3 per available block (pos / vel / att).
//
class UkfAidingSolution : public IUkfMeasurementModel {
 public:
  explicit UkfAidingSolution(UkfAidingSolutionOptions options = {});

  [[nodiscard]] bool CanHandle(const SensorMeasurement& measurement) const override;
  [[nodiscard]] int  MeasurementDim(const SensorMeasurement& measurement) const override;

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
  UkfAidingSolutionOptions options_;

  // Returns (has_pos, has_vel, has_att) for a given measurement
  static std::tuple<bool, bool, bool> ActiveBlocks(const core::AidingSolution& aid);
};

}  // namespace falconguide::estimation::ukf
