#pragma once

/**
 * @file echo_sounder.hpp
 * @brief EKF echo-sounder depth measurement model.
 */

#include "falconguide/estimation/backends/ekf/measurement_models/measurement_model.hpp"

#include <optional>

namespace falconguide::estimation::ekf {

/// @brief Options for EKF echo-sounder fusion.
struct EchoSounderOptions {
    // Sound speed correction factor.  depth_corrected = depth_raw * sound_speed /
    // 1500.
    double sound_speed_mps{1500.0};

    // Depth below the LTP z-origin (positive down).
    // h(x) = -p_enu.z + depth_origin_m
    double depth_origin_m{0.0};

    std::optional<double> sigma_m;

    // Mahalanobis gate (chi-squared, 1-DOF).  0 = disabled.
    double innovation_gate{0.0};
};

// ── EchoSounder
// ───────────────────────────────────────────────────────────────
//
// Scalar depth-below-surface update (underwater navigation).
//
// h(x) = -p_enu.z + depth_origin_m    (positive depth = below origin)
//
// H: [0,0,-1, 0,...] at position z column.
//
/// @brief EKF measurement model for scalar underwater depth updates.
class EchoSounder : public IMeasurementModel {
   public:
    explicit EchoSounder(EchoSounderOptions options = {});

    [[nodiscard]] bool CanHandle(const SensorMeasurement &measurement) const override;

    EstimatorUpdateResult Apply(NominalState &nominal, Eigen::VectorXd &error_state, Eigen::MatrixXd &covariance,
                                const StateLayout &layout, const SensorMeasurement &measurement,
                                const UpdateContext &context) override;

   private:
    EchoSounderOptions options_;
};

}  // namespace falconguide::estimation::ekf
