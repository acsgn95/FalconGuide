#pragma once

/**
 * @file wheel_odometry.hpp
 * @brief EKF wheel-odometry body-velocity measurement model.
 */

#include "falconguide/estimation/backends/ekf/measurement_models/measurement_model.hpp"

#include <Eigen/Core>
#include <optional>

namespace falconguide::estimation::ekf {

/// @brief Options for EKF wheel-odometry fusion.
struct WheelOdometryOptions {
    double sigma_linear_mps{0.1};
    double sigma_angular_radps{0.05};

    // Use angular rate from wheel odometry to constrain gyro bias.
    // Requires latest_gyro_radps to be updated externally.
    bool use_angular_rate{false};

    // Most recent IMU angular rate (rad/s), needed for angular rate update.
    // Update this before each AddMeasurement call when use_angular_rate is true.
    Eigen::Vector3d latest_gyro_radps{0.0, 0.0, 0.0};

    // Mahalanobis gate (chi-squared, 3-DOF for linear, 6-DOF if angular).  0 =
    // disabled.
    double innovation_gate{0.0};
};

// ── WheelOdometry
// ─────────────────────────────────────────────────────────────
//
// Body-frame linear (and optionally angular) velocity update.
//
// Linear measurement function:
//   h(x) = R_body_to_enu^T * v_enu
//
// H_vel = R^T                            (3×3 velocity block)
// H_att = SkewSymmetric(R^T * v_enu)     (3×3 attitude block)
//
// Angular update (when use_angular_rate is true):
//   h(x) = latest_gyro - gyro_bias
//   H_bg  = -I₃                          (3×3 gyro-bias block)
//
/// @brief EKF measurement model for body-frame wheel-odometry updates.
class WheelOdometry : public IMeasurementModel {
   public:
    explicit WheelOdometry(WheelOdometryOptions options = {});

    [[nodiscard]] bool CanHandle(const SensorMeasurement &measurement) const override;

    EstimatorUpdateResult Apply(NominalState &nominal, Eigen::VectorXd &error_state, Eigen::MatrixXd &covariance,
                                const StateLayout &layout, const SensorMeasurement &measurement,
                                const UpdateContext &context) override;

   private:
    WheelOdometryOptions options_;
};

}  // namespace falconguide::estimation::ekf
