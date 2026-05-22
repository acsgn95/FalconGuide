#pragma once

/**
 * @file range_finder.hpp
 * @brief EKF generic range-finder measurement model.
 */

#include "falconguide/estimation/backends/ekf/measurement_models/measurement_model.hpp"

#include <optional>

namespace falconguide::estimation::ekf {

/// @brief Options for EKF range-finder fusion.
struct RangeFinderOptions {
  // Expected range to target (m), used for innovation gate.  0 = disabled.
  double expected_range_m{0.0};

  // Treat the range as altitude above terrain (like a downward altimeter).
  // If false the measurement is ignored (range-to-obstacle has no simple h(x)).
  bool use_as_altitude{true};

  // Terrain elevation above LTP origin (m).
  double terrain_elevation_m{0.0};

  std::optional<double> sigma_m;

  // Mahalanobis gate (chi-squared, 1-DOF).  0 = disabled.
  double innovation_gate{0.0};
};

// ── RangeFinder
// ───────────────────────────────────────────────────────────────
//
// Scalar range update.  When use_as_altitude is true, the range is treated as
// altitude above terrain (identical measurement function to RadarAltimeter).
//
// h(x) = p_enu.z - terrain_elevation_m
//
/// @brief EKF measurement model for scalar range-as-altitude updates.
class RangeFinder : public IMeasurementModel {
public:
  explicit RangeFinder(RangeFinderOptions options = {});

  [[nodiscard]] bool
  CanHandle(const SensorMeasurement &measurement) const override;

  EstimatorUpdateResult Apply(NominalState &nominal,
                              Eigen::VectorXd &error_state,
                              Eigen::MatrixXd &covariance,
                              const StateLayout &layout,
                              const SensorMeasurement &measurement,
                              const UpdateContext &context) override;

private:
  RangeFinderOptions options_;
};

} // namespace falconguide::estimation::ekf
