#pragma once

#include "falconguide/core/navigation_state.hpp"
#include "falconguide/core/sensor_types.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <variant>

namespace falconguide::estimation {

using SensorMeasurement = std::variant<
    core::ImuMeasurement,
    core::MagnetometerMeasurement,
    core::BarometerMeasurement,
    core::RadarAltimeterMeasurement,
    core::WheelEncoderMeasurement,
    core::WheelOdometryMeasurement,
    core::AirspeedMeasurement,
    core::RangeFinderMeasurement,
    core::OpticalFlowMeasurement,
    core::DvlMeasurement,
    core::EchoSounderMeasurement,
    core::ExternalPoseMeasurement,
    core::ExternalVelocityMeasurement,
    core::ExternalOdometryMeasurement,
    core::GnssObservationEpoch,
    core::GnssSolution,
    core::CameraFrameMeasurement,
    core::GeoReferenceImageMeasurement,
    core::AidingSolution,
    core::StarTrackerMeasurement,
    core::GnssTightlyCoupledEpoch>;

enum class EstimatorBackend {
  Unknown,
  Ekf,
  CeresSlidingWindow,
  GtsamFactorGraph,
  Custom
};

enum class EstimatorUpdateResult {
  Accepted,
  Buffered,
  Rejected,
  OutOfOrder,
  NotInitialized,
  BackendError
};

// ── MeasurementUpdateReport ───────────────────────────────────────────────────
//
// Rich result returned by AddMeasurement().
// Provides enough information for diagnostics, logging, and health monitoring
// without requiring the caller to peek inside the filter internals.
//
struct MeasurementUpdateReport {
  EstimatorUpdateResult result{EstimatorUpdateResult::Rejected};

  // Name of the measurement model that handled (or attempted to handle) this
  // measurement.  Empty string when the measurement was buffered (IMU) or
  // no model matched.
  std::string_view model_name;

  // Euclidean norm of the state correction vector applied in this update
  // (||K·y|| for EKF, ||delta_x|| for UKF).  Populated only on Accepted.
  // Large values indicate a surprising or poorly-calibrated measurement.
  std::optional<double> correction_norm;
};

struct EstimatorOptions {
  EstimatorBackend backend{EstimatorBackend::Unknown};
  std::size_t max_buffered_measurements{0};
  bool enable_sensor_health_reporting{true};
  bool enable_covariance_output{true};
};

struct EstimatorInfo {
  EstimatorBackend backend{EstimatorBackend::Unknown};
  std::string name;
  std::string version;
};

class INavigationEstimator {
 public:
  virtual ~INavigationEstimator() = default;

  [[nodiscard]] virtual EstimatorInfo Info() const = 0;
  [[nodiscard]] virtual const EstimatorOptions& Options() const = 0;

  virtual MeasurementUpdateReport AddMeasurement(const SensorMeasurement& measurement) = 0;
  virtual EstimatorUpdateResult ProcessUntil(const core::Timestamp& timestamp) = 0;
  virtual void Reset() = 0;

  [[nodiscard]] virtual bool IsInitialized() const = 0;
  [[nodiscard]] virtual std::optional<core::NavigationState> LatestState() const = 0;
};

}  // namespace falconguide::estimation
