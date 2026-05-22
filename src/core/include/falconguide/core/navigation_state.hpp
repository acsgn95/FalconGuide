#pragma once

/**
 * @file navigation_state.hpp
 * @brief Unified navigation output state, quality, and sensor-health metadata.
 */

#include "falconguide/core/frames.hpp"
#include "falconguide/core/time.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <optional>

namespace falconguide::core {

/// @brief High-level health/status of the navigation solution.
enum class NavigationStatus {
    Unknown,         ///< Status is not known.
    NotInitialized,  ///< Estimator has not produced an initial state.
    Initializing,    ///< Estimator is collecting data required for initialization.
    Nominal,         ///< Navigation solution is healthy.
    Degraded,        ///< Solution is usable but quality has degraded.
    DeadReckoning,   ///< Estimator is propagating without recent aiding.
    Fault            ///< Solution is not reliable.
};

/// @brief Active estimator fusion mode.
enum class EstimatorMode {
    Unknown,             ///< Mode is not known.
    InertialOnly,        ///< IMU-only dead reckoning.
    VisualInertial,      ///< Visual-inertial fusion.
    GnssInertial,        ///< GNSS-inertial fusion.
    VisualInertialGnss,  ///< Visual-inertial-GNSS fusion.
    MultiSensorFusion    ///< More than the standard visual/GNSS/IMU set is active.
};

/// @brief Per-sensor contribution and health state.
enum class SensorHealth {
    Unknown,   ///< Health is not known.
    Healthy,   ///< Sensor is healthy and within expected residual limits.
    Degraded,  ///< Sensor is usable with reduced quality.
    Rejected,  ///< Latest measurements were rejected by validation or gating.
    Missing,   ///< Sensor is expected but absent.
    Stale,     ///< Sensor has not updated recently.
    Fault      ///< Sensor has reported or inferred a fault.
};

/// @brief Diagnostic status for a single sensor source.
struct SensorStatus {
    SensorHealth health{SensorHealth::Unknown};  ///< Current sensor health classification.
    bool used_in_solution{false};                ///< True if the sensor contributed to the latest state.
    std::optional<double> innovation_norm;       ///< Optional latest innovation norm.
    std::optional<double> last_update_age_s;     ///< Optional time since the last accepted update.
};

/// @brief Status summary for all raw, processed, and external navigation
/// sources.
struct NavigationSensorStatus {
    // ── Raw sensors ───────────────────────────────────────────────────────────
    SensorStatus imu;
    SensorStatus gnss;
    SensorStatus magnetometer;
    SensorStatus barometer;
    SensorStatus radar_altimeter;
    SensorStatus range_finder;
    SensorStatus optical_flow;
    SensorStatus wheel_odometry;
    SensorStatus airspeed;
    SensorStatus dvl;
    SensorStatus echo_sounder;

    // ── Processed / aiding solutions ─────────────────────────────────────────
    // Camera frame — raw image delivery status (not the processed solution).
    SensorStatus camera;
    // Processed algorithm outputs — each maps to one AidingSource.
    SensorStatus visual_odometry;           // PnP from camera frames
    SensorStatus geo_reference;             // camera + DEM image matching
    SensorStatus terrain_contour_matching;  // TERCOM: radar altimeter + DEM
    SensorStatus external_slam;

    // ── External pose / velocity injections ──────────────────────────────────
    SensorStatus external_pose;
    SensorStatus external_velocity;
    SensorStatus external_odometry;
};

struct NavigationQuality {
    bool initialized{false};                      ///< True after estimator initialization.
    bool degraded{false};                         ///< True when output quality is degraded.
    std::optional<double> position_accuracy_m;    ///< Optional 3D position accuracy estimate.
    std::optional<double> velocity_accuracy_mps;  ///< Optional 3D velocity accuracy estimate.
    std::optional<double> attitude_accuracy_rad;  ///< Optional attitude accuracy estimate.
    std::optional<double> horizontal_accuracy_m;  ///< Optional horizontal position accuracy.
    std::optional<double> vertical_accuracy_m;    ///< Optional vertical position accuracy.
};

/// @brief Complete navigation solution emitted by estimators and pipelines.
struct NavigationState {
    Timestamp timestamp;  ///< State timestamp.

    Vec3<EcefFrame> position_ecef_m;    ///< Position in ECEF meters.
    Vec3<EcefFrame> velocity_ecef_mps;  ///< Velocity in ECEF meters per second.
    Vec3<EnuFrame> position_enu_m;      ///< Position in local ENU meters.
    Vec3<EnuFrame> velocity_enu_mps;    ///< Velocity in local ENU meters per second.

    Eigen::Quaterniond orientation_body_to_enu{Eigen::Quaterniond::Identity()};  ///< Body-to-ENU attitude.
    Vec3<BodyFrame> angular_rate_body_radps;                                     ///< Body angular rate in rad/s.
    Vec3<BodyFrame> specific_force_body_mps2;                                    ///< Body specific force in m/s^2.

    Eigen::Matrix<double, 15, 15> covariance{Eigen::Matrix<double, 15, 15>::Zero()};  ///< Core 15-state covariance.

    NavigationStatus status{NavigationStatus::Unknown};  ///< High-level solution status.
    EstimatorMode mode{EstimatorMode::Unknown};          ///< Active fusion mode.
    NavigationQuality quality;                           ///< Quality and accuracy metadata.
    NavigationSensorStatus sensors;                      ///< Per-sensor status metadata.
};

}  // namespace falconguide::core
