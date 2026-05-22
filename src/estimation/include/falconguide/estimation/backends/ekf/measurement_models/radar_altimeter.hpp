#pragma once

/**
 * @file radar_altimeter.hpp
 * @brief EKF radar-altimeter height-above-terrain measurement model.
 */

#include "falconguide/estimation/backends/ekf/measurement_models/measurement_model.hpp"

#include <optional>

namespace falconguide::estimation::ekf {

/// @brief Options for EKF radar-altimeter fusion.
struct RadarAltimeterOptions {
    // Terrain elevation above the LTP origin (m).  Use 0.0 if origin is at
    // terrain level.
    double terrain_elevation_m{0.0};

    std::optional<double> sigma_m;

    // Mahalanobis gate (chi-squared, 1-DOF).  0 = disabled.
    double innovation_gate{0.0};
};

// ── RadarAltimeter
// ────────────────────────────────────────────────────────────
//
// Scalar altitude-above-terrain update using radar/lidar altimeter range.
//
// Measurement function (flat-Earth):
//   h(x) = p_enu.z - terrain_elevation_m
//
// H: [0,0,1, 0,...] at position z column.
//
/// @brief EKF measurement model for scalar altitude-above-terrain updates.
class RadarAltimeter : public IMeasurementModel {
   public:
    explicit RadarAltimeter(RadarAltimeterOptions options = {});

    [[nodiscard]] bool CanHandle(const SensorMeasurement &measurement) const override;

    EstimatorUpdateResult Apply(NominalState &nominal, Eigen::VectorXd &error_state, Eigen::MatrixXd &covariance,
                                const StateLayout &layout, const SensorMeasurement &measurement,
                                const UpdateContext &context) override;

   private:
    RadarAltimeterOptions options_;
};

}  // namespace falconguide::estimation::ekf
