#pragma once

/**
 * @file barometer.hpp
 * @brief EKF altitude update from barometer pressure or altitude.
 */

#include "falconguide/estimation/backends/ekf/measurement_models/measurement_model.hpp"

#include <optional>

namespace falconguide::estimation::ekf {

/// @brief Options for EKF barometer fusion.
struct BarometerOptions {
    // Override altitude noise.  std::nullopt → derive from pressure measurement.
    std::optional<double> altitude_sigma_m;

    // ISA sea-level pressure used for pressure→altitude conversion (Pa).
    double isa_sea_level_pressure_pa{101325.0};

    // Mahalanobis gate (chi-squared threshold, 1-DOF).  0 = disabled.
    double innovation_gate{0.0};
};

// ── Barometer
// ─────────────────────────────────────────────────────────────────
//
// Altitude-only scalar update.
//
// Observation:   z = altitude_m above the LTP origin (ENU z-axis).
//   • Uses measurement.altitude_m if populated by the driver.
//   • Falls back to ISA pressure-altitude formula otherwise.
//
// Measurement function:
//   h(x) = p_enu.z           (if BaroBias not in layout)
//   h(x) = p_enu.z + b_baro  (if BaroBias is in layout)
//
// H: [0,0,1, 0,..., (1 at BaroBias if present)]
//
/// @brief EKF measurement model for scalar altitude updates.
class Barometer : public IMeasurementModel {
   public:
    explicit Barometer(BarometerOptions options = {});

    [[nodiscard]] bool CanHandle(const SensorMeasurement &measurement) const override;

    EstimatorUpdateResult Apply(NominalState &nominal, Eigen::VectorXd &error_state, Eigen::MatrixXd &covariance,
                                const StateLayout &layout, const SensorMeasurement &measurement,
                                const UpdateContext &context) override;

   private:
    BarometerOptions options_;
};

}  // namespace falconguide::estimation::ekf
