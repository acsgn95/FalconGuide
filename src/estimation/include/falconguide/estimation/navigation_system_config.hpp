#pragma once

/**
 * @file navigation_system_config.hpp
 * @brief Top-level configuration for navigation-system construction.
 */

#include "falconguide/estimation/backends/ekf/ekf_state.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/aiding_solution.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/airspeed.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/barometer.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/dvl.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/echo_sounder.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/external_odometry.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/external_pose.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/external_velocity.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/gnss_loosely_coupled.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/gnss_tightly_coupled.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/magnetometer.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/optical_flow.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/radar_altimeter.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/range_finder.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/star_tracker.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/wheel_odometry.hpp"
#include "falconguide/estimation/backends/ukf/ukf_state.hpp"

namespace falconguide::estimation {

/// @brief Backend selection exposed by NavigationSystemConfig.
enum class EstimatorBackendChoice {
    Ekf,    ///< Error-state EKF backend.
    Ukf,    ///< Unscented Kalman Filter backend.
    Ceres,  ///< Ceres sliding-window smoother backend.
    Gtsam   ///< GTSAM factor-graph smoother backend.
};

// ── Per-sensor config blocks
// ──────────────────────────────────────────────────
//
// Each block has an enabled flag plus the relevant model options.  Disabled
// sensors are zero-cost: no model is registered and no state segment is added.
//
// Pass NavigationSystemConfig to NavigationSystem and the factory wires
// everything automatically.
//

struct ImuSensorConfig {
    ekf::ImuNoiseModel noise;  ///< IMU process-noise configuration.
};

// ─────────────────────────────────────────────────────────────────────────────

enum class GnssIntegrationMode {
    LooselyCoupled,  ///< Fuses receiver-computed GnssSolution position and
                     ///< velocity.
    TightlyCoupled,  ///< Fuses raw pseudorange and Doppler epochs.
};

/// @brief GNSS registration and measurement-model configuration.
struct GnssSensorConfig {
    bool enabled{false};                                            ///< Enables GNSS fusion.
    GnssIntegrationMode mode{GnssIntegrationMode::LooselyCoupled};  ///< GNSS fusion mode.
    ekf::GnssLooselyCoupledOptions loosely_coupled;                 ///< Loosely coupled model options.
    ekf::GnssTightlyCoupledOptions tightly_coupled;                 ///< Tightly coupled model options.
};

// ─────────────────────────────────────────────────────────────────────────────

struct BarometerSensorConfig {
    bool enabled{false};            ///< Enables barometer fusion.
    ekf::BarometerOptions options;  ///< Barometer model options.
};

// ─────────────────────────────────────────────────────────────────────────────

struct MagnetometerSensorConfig {
    bool enabled{false};               ///< Enables magnetometer fusion.
    ekf::MagnetometerOptions options;  ///< Magnetometer model options.
};

// ─────────────────────────────────────────────────────────────────────────────

struct StarTrackerSensorConfig {
    bool enabled{false};              ///< Enables star-tracker fusion.
    ekf::StarTrackerOptions options;  ///< Star-tracker model options.
};

// ─────────────────────────────────────────────────────────────────────────────

struct WheelOdometrySensorConfig {
    bool enabled{false};                ///< Enables wheel-odometry fusion.
    ekf::WheelOdometryOptions options;  ///< Wheel-odometry model options.
};

// ─────────────────────────────────────────────────────────────────────────────

struct RadarAltimeterSensorConfig {
    bool enabled{false};                 ///< Enables radar-altimeter fusion.
    ekf::RadarAltimeterOptions options;  ///< Radar-altimeter model options.
};

// ─────────────────────────────────────────────────────────────────────────────

struct RangeFinderSensorConfig {
    bool enabled{false};              ///< Enables range-finder fusion.
    ekf::RangeFinderOptions options;  ///< Range-finder model options.
};

// ─────────────────────────────────────────────────────────────────────────────

struct OpticalFlowSensorConfig {
    bool enabled{false};              ///< Enables optical-flow fusion.
    ekf::OpticalFlowOptions options;  ///< Optical-flow model options.
};

// ─────────────────────────────────────────────────────────────────────────────

struct DvlSensorConfig {
    bool enabled{false};      ///< Enables DVL fusion.
    ekf::DvlOptions options;  ///< DVL model options.
};

// ─────────────────────────────────────────────────────────────────────────────

struct EchoSounderSensorConfig {
    bool enabled{false};              ///< Enables echo-sounder fusion.
    ekf::EchoSounderOptions options;  ///< Echo-sounder model options.
};

// ─────────────────────────────────────────────────────────────────────────────

struct AirspeedSensorConfig {
    bool enabled{false};           ///< Enables airspeed fusion.
    ekf::AirspeedOptions options;  ///< Airspeed model options.
};

// ─────────────────────────────────────────────────────────────────────────────

struct ExternalPoseSensorConfig {
    bool enabled{false};               ///< Enables external-pose fusion.
    ekf::ExternalPoseOptions options;  ///< External-pose model options.
};

// ─────────────────────────────────────────────────────────────────────────────

struct ExternalVelocitySensorConfig {
    bool enabled{false};                   ///< Enables external-velocity fusion.
    ekf::ExternalVelocityOptions options;  ///< External-velocity model options.
};

// ─────────────────────────────────────────────────────────────────────────────

struct ExternalOdometrySensorConfig {
    bool enabled{false};                   ///< Enables external-odometry fusion.
    ekf::ExternalOdometryOptions options;  ///< External-odometry model options.
};

// ─────────────────────────────────────────────────────────────────────────────

struct AidingSolutionSensorConfig {
    bool enabled{false};                 ///< Enables generic aiding-solution fusion.
    ekf::AidingSolutionOptions options;  ///< Aiding-solution model options.
};

// ── NavigationSystemConfig
// ────────────────────────────────────────────────────
//
// Complete system configuration.  Write one instance per hardware platform and
// pass it to NavigationSystem.
//
// Example (UAV with u-blox F9P + ICM-42688 + MS5611 + optical flow):
//
//   NavigationSystemConfig cfg;
//   cfg.imu.noise.accel_noise_density_mps2_per_sqrthz = 0.0028;
//   cfg.gnss.enabled = true;
//   cfg.barometer.enabled = true;
//   cfg.optical_flow.enabled = true;
//   NavigationSystem nav(cfg);
//
struct NavigationSystemConfig {
    ImuSensorConfig imu;                             ///< IMU propagation configuration.
    GnssSensorConfig gnss;                           ///< GNSS fusion configuration.
    BarometerSensorConfig barometer;                 ///< Barometer fusion configuration.
    MagnetometerSensorConfig magnetometer;           ///< Magnetometer fusion configuration.
    StarTrackerSensorConfig star_tracker;            ///< Star-tracker fusion configuration.
    WheelOdometrySensorConfig wheel_odometry;        ///< Wheel-odometry fusion configuration.
    RadarAltimeterSensorConfig radar_altimeter;      ///< Radar-altimeter fusion configuration.
    RangeFinderSensorConfig range_finder;            ///< Range-finder fusion configuration.
    OpticalFlowSensorConfig optical_flow;            ///< Optical-flow fusion configuration.
    DvlSensorConfig dvl;                             ///< DVL fusion configuration.
    EchoSounderSensorConfig echo_sounder;            ///< Echo-sounder fusion configuration.
    AirspeedSensorConfig airspeed;                   ///< Airspeed fusion configuration.
    ExternalPoseSensorConfig external_pose;          ///< External-pose fusion configuration.
    ExternalVelocitySensorConfig external_velocity;  ///< External-velocity fusion configuration.
    ExternalOdometrySensorConfig external_odometry;  ///< External-odometry fusion configuration.
    AidingSolutionSensorConfig aiding_solution;      ///< Generic aiding fusion configuration.

    // Backend selection
    EstimatorBackendChoice backend{EstimatorBackendChoice::Ekf};  ///< Backend selected by the system factory.

    // UKF-only: Merwe scaled sigma point parameters.
    // Ignored when backend == Ekf.
    ukf::MerweSigmaParams ukf_sigma_params;  ///< UKF sigma-point parameters.

    // EKF engine tuning
    double dead_reckoning_threshold_s{5.0};  ///< Aiding gap that marks output as dead reckoning.
    double min_imu_dt_s{1e-6};               ///< Minimum accepted IMU propagation interval.
    double max_imu_dt_s{0.05};               ///< Maximum IMU propagation step before splitting or rejection.
};

}  // namespace falconguide::estimation
