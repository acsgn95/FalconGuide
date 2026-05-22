#pragma once

/**
 * @file dvl.hpp
 * @brief EKF Doppler Velocity Log measurement model.
 */

#include "falconguide/estimation/backends/ekf/measurement_models/measurement_model.hpp"

#include <Eigen/Core>
#include <optional>

namespace falconguide::estimation::ekf {

/// @brief Options for EKF DVL fusion.
struct DvlOptions {
    // Rotation from DVL sensor frame to body frame.
    // Identity means DVL axes are aligned with body axes.
    Eigen::Matrix3d dvl_to_body_rotation{Eigen::Matrix3d::Identity()};

    // Whether to use only bottom-track pings (reject water-track returns).
    bool bottom_track_only{false};

    std::optional<double> sigma_mps;

    // Mahalanobis gate (chi-squared, 3-DOF).  0 = disabled.
    double innovation_gate{0.0};
};

// ── Dvl
// ───────────────────────────────────────────────────────────────────────
//
// Doppler Velocity Log 3-axis velocity update (underwater).
//
// Measurement function (velocity in DVL frame):
//   h(x) = R_dvl_to_body^T * R_body_to_enu^T * v_enu
//
// H:
//   H_vel = A = R_dvl_to_body^T * R^T              (3×3 velocity block)
//   H_att = A * SkewSymmetric(R^T * v_enu)          (3×3 attitude block, via
//   chain rule)
//
/// @brief EKF measurement model for 3-axis DVL velocity updates.
class Dvl : public IMeasurementModel {
   public:
    explicit Dvl(DvlOptions options = {});

    [[nodiscard]] bool CanHandle(const SensorMeasurement &measurement) const override;

    EstimatorUpdateResult Apply(NominalState &nominal, Eigen::VectorXd &error_state, Eigen::MatrixXd &covariance,
                                const StateLayout &layout, const SensorMeasurement &measurement,
                                const UpdateContext &context) override;

   private:
    DvlOptions options_;
};

}  // namespace falconguide::estimation::ekf
