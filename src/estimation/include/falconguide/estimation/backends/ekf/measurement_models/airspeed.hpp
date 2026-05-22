#pragma once

/**
 * @file airspeed.hpp
 * @brief EKF scalar airspeed measurement model.
 */

#include "falconguide/estimation/backends/ekf/measurement_models/measurement_model.hpp"

#include <Eigen/Core>

namespace falconguide::estimation::ekf {

/// @brief Options for EKF airspeed fusion.
struct AirspeedOptions {
    // Known wind velocity in ENU frame (m/s).  Zero for calm conditions.
    Eigen::Vector3d wind_enu_mps{0.0, 0.0, 0.0};

    double sigma_mps{0.5};

    // Mahalanobis gate (chi-squared, 1-DOF).  0 = disabled.
    double innovation_gate{0.0};
};

// ── Airspeed
// ──────────────────────────────────────────────────────────────────
//
// Scalar airspeed magnitude update.
//
// Measurement function:
//   v_air_body = R^T * (v_enu - v_wind)
//   h(x) = ||v_air_body||
//
// Jacobian:
//   H_vel = (v_air_body / h)^T * R^T               (1×3 velocity block)
//   H_att = (v_air_body / h)^T * SkewSymmetric(v_air_body)  (1×3 attitude
//   block)
//
/// @brief EKF measurement model for airspeed magnitude updates.
class Airspeed : public IMeasurementModel {
   public:
    explicit Airspeed(AirspeedOptions options = {});

    [[nodiscard]] bool CanHandle(const SensorMeasurement &measurement) const override;

    EstimatorUpdateResult Apply(NominalState &nominal, Eigen::VectorXd &error_state, Eigen::MatrixXd &covariance,
                                const StateLayout &layout, const SensorMeasurement &measurement,
                                const UpdateContext &context) override;

   private:
    AirspeedOptions options_;
};

}  // namespace falconguide::estimation::ekf
