#pragma once

/**
 * @file ekf_state.hpp
 * @brief Error-state EKF nominal state, process noise, and covariance state.
 */

#include "falconguide/core/frames.hpp"
#include "falconguide/core/time.hpp"
#include "falconguide/estimation/backends/ekf/state_layout.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <optional>

namespace falconguide::estimation::ekf {

// ── NominalState
// ──────────────────────────────────────────────────────────────
//
// The "large" reference trajectory propagated between updates.
// Optional fields are populated only when the corresponding StateSegmentId is
// active in the layout (e.g. gnss_clock is non-null only when GnssClock is
// registered, which is required for tightly coupled GNSS).
//
struct NominalState {
    core::Timestamp timestamp;  ///< State timestamp.

    core::Vec3<core::EnuFrame> position_enu_m;                                   ///< Nominal ENU position.
    core::Vec3<core::EnuFrame> velocity_enu_mps;                                 ///< Nominal ENU velocity.
    Eigen::Quaterniond orientation_body_to_enu{Eigen::Quaterniond::Identity()};  ///< Nominal body-to-ENU attitude.

    core::Vec3<core::ImuFrame> accel_bias_mps2;  ///< Accelerometer bias estimate.
    core::Vec3<core::ImuFrame> gyro_bias_radps;  ///< Gyroscope bias estimate.

    // [0]: receiver clock bias (m), [1]: clock drift (m/s).
    // Populated only when StateSegmentId::GnssClock is active.
    std::optional<Eigen::Vector2d> gnss_clock;  ///< Optional receiver clock bias and drift state.
};

// ── ImuNoiseModel
// ─────────────────────────────────────────────────────────────
//
// Continuous-time spectral densities.  Units match the standard IMU datasheet
// notation: noise density (random noise) and random walk (bias instability).
//
struct ImuNoiseModel {
    double accel_noise_density_mps2_per_sqrthz{3.0e-3};  ///< Accelerometer white-noise density.
    double gyro_noise_density_radps_per_sqrthz{1.5e-4};  ///< Gyroscope white-noise density.
    double accel_random_walk_mps3_per_sqrthz{3.0e-5};    ///< Accelerometer bias random walk.
    double gyro_random_walk_radps2_per_sqrthz{2.0e-6};   ///< Gyroscope bias random walk.
};

// ── EkfState
// ──────────────────────────────────────────────────────────────────
//
// Complete dynamic EKF state: nominal trajectory + error-state distribution.
//
// The error_state vector is always near zero — it is reset to zero after each
// measurement update injects the correction into the nominal state.
// The covariance grows during IMU propagation and shrinks during updates.
//
struct EkfState {
    NominalState nominal;         ///< Current nominal trajectory state.
    Eigen::VectorXd error_state;  ///< Error-state mean, reset after injection.
    Eigen::MatrixXd covariance;   ///< Error-state covariance.
    StateLayout layout;           ///< Dynamic error-state segment layout.
};

}  // namespace falconguide::estimation::ekf
