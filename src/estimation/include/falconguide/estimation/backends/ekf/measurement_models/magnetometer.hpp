#pragma once

/**
 * @file magnetometer.hpp
 * @brief EKF magnetometer attitude-heading measurement model.
 */

#include "falconguide/estimation/backends/ekf/measurement_models/measurement_model.hpp"

#include <Eigen/Core>
#include <optional>

namespace falconguide::estimation::ekf {

/// @brief Options for EKF magnetometer fusion.
struct MagnetometerOptions {
    // Local reference magnetic field in ENU frame (Tesla).
    // Obtain from the World Magnetic Model (WMM) for the operating area.
    // NED→ENU conversion: enu = [ned.y, ned.x, -ned.z].
    Eigen::Vector3d reference_field_enu_tesla{0.0, 2.0e-5, -4.3e-5};  // ~mid-latitude default

    // Override measurement noise. std::nullopt → isotropic (sigma_tesla²).
    std::optional<double> sigma_tesla;

    // Mahalanobis gate (chi-squared, 3-DOF).  0 = disabled.
    double innovation_gate{0.0};
};

// ── Magnetometer
// ──────────────────────────────────────────────────────────────
//
// Full 3-axis magnetic field update.
//
// Measurement function:
//   h(x) = R_body_to_enu^T * b_ref_enu
//
// Jacobian (attitude block only, 3×3):
//   dh/dδθ = SkewSymmetric(h_pred)
//
/// @brief EKF measurement model for 3-axis magnetic-field updates.
class Magnetometer : public IMeasurementModel {
   public:
    explicit Magnetometer(MagnetometerOptions options = {});

    [[nodiscard]] bool CanHandle(const SensorMeasurement &measurement) const override;

    EstimatorUpdateResult Apply(NominalState &nominal, Eigen::VectorXd &error_state, Eigen::MatrixXd &covariance,
                                const StateLayout &layout, const SensorMeasurement &measurement,
                                const UpdateContext &context) override;

   private:
    MagnetometerOptions options_;
};

}  // namespace falconguide::estimation::ekf
