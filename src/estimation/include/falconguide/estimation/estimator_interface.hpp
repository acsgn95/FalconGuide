#pragma once

/**
 * @file estimator_interface.hpp
 * @brief Backend-neutral estimator interface and measurement variant.
 */

#include "falconguide/core/navigation_state.hpp"
#include "falconguide/core/sensor_types.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <variant>

namespace falconguide::estimation {

/// @brief Variant containing every sensor measurement type accepted by
/// estimators.
using SensorMeasurement =
    std::variant<core::ImuMeasurement, core::MagnetometerMeasurement, core::BarometerMeasurement,
                 core::RadarAltimeterMeasurement, core::WheelEncoderMeasurement, core::WheelOdometryMeasurement,
                 core::AirspeedMeasurement, core::RangeFinderMeasurement, core::OpticalFlowMeasurement,
                 core::DvlMeasurement, core::EchoSounderMeasurement, core::ExternalPoseMeasurement,
                 core::ExternalVelocityMeasurement, core::ExternalOdometryMeasurement, core::GnssObservationEpoch,
                 core::GnssSolution, core::CameraFrameMeasurement, core::GeoReferenceImageMeasurement,
                 core::AidingSolution, core::StarTrackerMeasurement, core::GnssTightlyCoupledEpoch>;

/// @brief Concrete estimator backend identity.
enum class EstimatorBackend {
    Unknown,             ///< Backend is not known.
    Ekf,                 ///< Error-state Extended Kalman Filter backend.
    CeresSlidingWindow,  ///< Ceres nonlinear sliding-window backend.
    GtsamFactorGraph,    ///< GTSAM factor-graph backend.
    Custom               ///< Custom backend not otherwise enumerated.
};

/// @brief Result of submitting a measurement or processing to a timestamp.
enum class EstimatorUpdateResult {
    Accepted,        ///< Measurement was fused or processing completed.
    Buffered,        ///< Measurement was stored for later processing.
    Rejected,        ///< Measurement was validly refused, e.g. by gating.
    OutOfOrder,      ///< Measurement timestamp violates ordering requirements.
    NotInitialized,  ///< Backend cannot process the request before initialization.
    BackendError     ///< Backend encountered an internal error.
};

// ── MeasurementUpdateReport
// ───────────────────────────────────────────────────
//
// Rich result returned by AddMeasurement().
// Provides enough information for diagnostics, logging, and health monitoring
// without requiring the caller to peek inside the filter internals.
//
struct MeasurementUpdateReport {
    EstimatorUpdateResult result{EstimatorUpdateResult::Rejected};  ///< High-level update outcome.

    // Name of the measurement model that handled (or attempted to handle) this
    // measurement.  Empty string when the measurement was buffered (IMU) or
    // no model matched.
    std::string_view model_name;  ///< Measurement model that handled the update,
                                  ///< when available.

    // Euclidean norm of the state correction vector applied in this update
    // (||K·y|| for EKF, ||delta_x|| for UKF).  Populated only on Accepted.
    // Large values indicate a surprising or poorly-calibrated measurement.
    std::optional<double> correction_norm;  ///< Optional state-correction norm
                                            ///< applied by the backend.
};

/// @brief Backend-independent estimator options.
struct EstimatorOptions {
    EstimatorBackend backend{EstimatorBackend::Unknown};  ///< Backend selected for this estimator.
    std::size_t max_buffered_measurements{0};             ///< Maximum queued or buffered measurements.
    bool enable_sensor_health_reporting{true};            ///< Enables population of per-sensor status fields.
    bool enable_covariance_output{true};                  ///< Enables covariance export in NavigationState.
};

/// @brief Static metadata reported by an estimator backend.
struct EstimatorInfo {
    EstimatorBackend backend{EstimatorBackend::Unknown};  ///< Backend type.
    std::string name;                                     ///< Human-readable backend name.
    std::string version;                                  ///< Backend implementation version.
};

/// @brief Common interface implemented by all navigation estimators.
class INavigationEstimator {
   public:
    /// @brief Virtual destructor for interface use.
    virtual ~INavigationEstimator() = default;

    /// @brief Returns backend metadata.
    [[nodiscard]] virtual EstimatorInfo Info() const = 0;
    /// @brief Returns immutable backend options.
    [[nodiscard]] virtual const EstimatorOptions &Options() const = 0;

    /// @brief Adds one sensor measurement to the estimator.
    /// @param measurement Typed measurement variant.
    /// @return Detailed update report.
    virtual MeasurementUpdateReport AddMeasurement(const SensorMeasurement &measurement) = 0;
    /// @brief Processes all buffered data up to a target timestamp.
    virtual EstimatorUpdateResult ProcessUntil(const core::Timestamp &timestamp) = 0;
    /// @brief Clears backend state and returns to an uninitialized state.
    virtual void Reset() = 0;

    /// @brief Returns true when a navigation state is initialized.
    [[nodiscard]] virtual bool IsInitialized() const = 0;
    /// @brief Returns the latest navigation state, if available.
    [[nodiscard]] virtual std::optional<core::NavigationState> LatestState() const = 0;
};

}  // namespace falconguide::estimation
