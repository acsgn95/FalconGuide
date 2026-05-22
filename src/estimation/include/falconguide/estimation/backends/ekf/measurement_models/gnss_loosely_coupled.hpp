#pragma once

/**
 * @file gnss_loosely_coupled.hpp
 * @brief EKF loosely-coupled GNSS solution measurement model.
 */

#include "falconguide/estimation/backends/ekf/measurement_models/measurement_model.hpp"

#include <optional>

namespace falconguide::estimation::ekf {

// ── GnssLooselyCoupledOptions
// ─────────────────────────────────────────────────
/// @brief Options for EKF loosely-coupled GNSS fusion.
struct GnssLooselyCoupledOptions {
    // Override the position noise from GnssSolution.covariance (m, 1-sigma).
    std::optional<double> position_sigma_m;

    // Override the velocity noise from GnssSolution.covariance (m/s, 1-sigma).
    std::optional<double> velocity_sigma_mps;

    // Mahalanobis distance gate for the position innovation (chi-squared, DOF=3).
    // 0 means no gating.  A value of 7.815 corresponds to 95% acceptance.
    double position_gate{0.0};
};

// ── GnssLooselyCoupled
// ────────────────────────────────────────────────────────
//
// Fuses a GnssSolution (solver-level position + velocity) into the EKF.
//
// Observation model (6×N stacked):
//
//   z_pos = p_gnss_enu  →  H_pos = [I₃  0₃  0₃  …]  (selects δp)
//   z_vel = v_gnss_enu  →  H_vel = [0₃  I₃  0₃  …]  (selects δv)
//
// Both ECEF quantities are rotated to the ENU local-tangent-plane before use.
// The local-tangent-plane itself is owned by the engine and passed through
// UpdateContext; the model returns NotInitialized if the LTP is not yet set.
//
// Covariances are taken from GnssSolution (rotated to ENU) by default;
// GnssLooselyCoupledOptions allow fixed overrides for testing or conservative
// noise floors.
//
/// @brief EKF measurement model for receiver-computed GNSS position and
/// velocity.
class GnssLooselyCoupled : public IMeasurementModel {
   public:
    explicit GnssLooselyCoupled(GnssLooselyCoupledOptions options = {});

    [[nodiscard]] bool CanHandle(const SensorMeasurement &measurement) const override;

    EstimatorUpdateResult Apply(NominalState &nominal, Eigen::VectorXd &error_state, Eigen::MatrixXd &covariance,
                                const StateLayout &layout, const SensorMeasurement &measurement,
                                const UpdateContext &context) override;

   private:
    EstimatorUpdateResult ApplySolution(NominalState &nominal, Eigen::VectorXd &error_state,
                                        Eigen::MatrixXd &covariance, const StateLayout &layout,
                                        const core::GnssSolution &gnss, const core::LocalTangentPlane &ltp);

    GnssLooselyCoupledOptions options_;
};

}  // namespace falconguide::estimation::ekf
