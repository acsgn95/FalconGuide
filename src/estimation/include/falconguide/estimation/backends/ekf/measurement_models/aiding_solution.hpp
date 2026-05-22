#pragma once

/**
 * @file aiding_solution.hpp
 * @brief EKF measurement model for generic aided-navigation solutions.
 */

#include "falconguide/estimation/backends/ekf/measurement_models/measurement_model.hpp"

#include <optional>

namespace falconguide::estimation::ekf {

/// @brief Options for EKF aided-solution fusion.
struct AidingSolutionOptions {
  // Reject measurements whose match_score is below this threshold.  0 = accept
  // all.
  double min_match_score{0.0};

  // Override position / velocity / attitude noise when provided.
  std::optional<double> position_sigma_m;
  std::optional<double> velocity_sigma_mps;
  std::optional<double> attitude_sigma_rad;

  // Mahalanobis gate (chi-squared, applied per sub-block).  0 = disabled.
  double innovation_gate{0.0};
};

// ── AidingSolution
// ────────────────────────────────────────────────────────────
//
// Generic update from any terrain-aided, image-aided, or SLAM pipeline.
//
// Selectively applies position, velocity, and attitude updates based on which
// optional fields are populated in AidingSolution, and which axes are marked
// valid by orientation_validity.
//
// Position  (ECEF → ENU):  H_pos = I₃                   (3×3 position block)
// Velocity  (ECEF → ENU):  H_vel = I₃                   (3×3 velocity block)
// Attitude (body→ECEF → body→ENU via LTP):
//   δθ = LogMapSo3(q_pred^{−1} ⊗ q_obs)
//   H_att = I₃ (selected axes from orientation_validity)
//
/// @brief EKF measurement model for terrain, image, visual-odometry, and SLAM
/// aiding.
class AidingSolution : public IMeasurementModel {
public:
  explicit AidingSolution(AidingSolutionOptions options = {});

  [[nodiscard]] bool
  CanHandle(const SensorMeasurement &measurement) const override;

  EstimatorUpdateResult Apply(NominalState &nominal,
                              Eigen::VectorXd &error_state,
                              Eigen::MatrixXd &covariance,
                              const StateLayout &layout,
                              const SensorMeasurement &measurement,
                              const UpdateContext &context) override;

private:
  AidingSolutionOptions options_;
};

} // namespace falconguide::estimation::ekf
