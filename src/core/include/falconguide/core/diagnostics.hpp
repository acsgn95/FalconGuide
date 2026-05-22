#pragma once

/**
 * @file diagnostics.hpp
 * @brief String conversion helpers for diagnostics, logging, and telemetry.
 */

#include "falconguide/core/navigation_state.hpp"
#include "falconguide/core/sensors/aiding.hpp"
#include "falconguide/core/sensors/common.hpp"
#include "falconguide/core/sensors/gnss.hpp"
#include "falconguide/estimation/estimator_interface.hpp"

#include <ostream>
#include <string_view>

namespace falconguide::core {

// ── NavigationStatus ─────────────────────────────────────────────────────────

/// @brief Converts NavigationStatus to a stable diagnostic string.
constexpr std::string_view ToString(NavigationStatus s) noexcept {
  switch (s) {
  case NavigationStatus::Unknown:
    return "Unknown";
  case NavigationStatus::NotInitialized:
    return "NotInitialized";
  case NavigationStatus::Initializing:
    return "Initializing";
  case NavigationStatus::Nominal:
    return "Nominal";
  case NavigationStatus::Degraded:
    return "Degraded";
  case NavigationStatus::DeadReckoning:
    return "DeadReckoning";
  case NavigationStatus::Fault:
    return "Fault";
  }
  return "Unknown";
}

/// @brief Streams a NavigationStatus as text.
inline std::ostream &operator<<(std::ostream &os, NavigationStatus s) {
  return os << ToString(s);
}

// ── EstimatorMode
// ─────────────────────────────────────────────────────────────

/// @brief Converts EstimatorMode to a stable diagnostic string.
constexpr std::string_view ToString(EstimatorMode m) noexcept {
  switch (m) {
  case EstimatorMode::Unknown:
    return "Unknown";
  case EstimatorMode::InertialOnly:
    return "InertialOnly";
  case EstimatorMode::VisualInertial:
    return "VisualInertial";
  case EstimatorMode::GnssInertial:
    return "GnssInertial";
  case EstimatorMode::VisualInertialGnss:
    return "VisualInertialGnss";
  case EstimatorMode::MultiSensorFusion:
    return "MultiSensorFusion";
  }
  return "Unknown";
}

/// @brief Streams an EstimatorMode as text.
inline std::ostream &operator<<(std::ostream &os, EstimatorMode m) {
  return os << ToString(m);
}

// ── SensorHealth
// ──────────────────────────────────────────────────────────────

/// @brief Converts SensorHealth to a stable diagnostic string.
constexpr std::string_view ToString(SensorHealth h) noexcept {
  switch (h) {
  case SensorHealth::Unknown:
    return "Unknown";
  case SensorHealth::Healthy:
    return "Healthy";
  case SensorHealth::Degraded:
    return "Degraded";
  case SensorHealth::Rejected:
    return "Rejected";
  case SensorHealth::Missing:
    return "Missing";
  case SensorHealth::Stale:
    return "Stale";
  case SensorHealth::Fault:
    return "Fault";
  }
  return "Unknown";
}

/// @brief Streams a SensorHealth as text.
inline std::ostream &operator<<(std::ostream &os, SensorHealth h) {
  return os << ToString(h);
}

// ── MeasurementValidity
// ───────────────────────────────────────────────────────

/// @brief Converts MeasurementValidity to a stable diagnostic string.
constexpr std::string_view ToString(MeasurementValidity v) noexcept {
  switch (v) {
  case MeasurementValidity::Valid:
    return "Valid";
  case MeasurementValidity::Saturated:
    return "Saturated";
  case MeasurementValidity::Dropped:
    return "Dropped";
  case MeasurementValidity::OutOfOrder:
    return "OutOfOrder";
  case MeasurementValidity::Unknown:
    return "Unknown";
  }
  return "Unknown";
}

/// @brief Streams a MeasurementValidity as text.
inline std::ostream &operator<<(std::ostream &os, MeasurementValidity v) {
  return os << ToString(v);
}

// ── GnssFixType
// ───────────────────────────────────────────────────────────────

/// @brief Converts GnssFixType to a stable diagnostic string.
constexpr std::string_view ToString(GnssFixType f) noexcept {
  switch (f) {
  case GnssFixType::NoFix:
    return "NoFix";
  case GnssFixType::Single:
    return "Single";
  case GnssFixType::Differential:
    return "Differential";
  case GnssFixType::RtkFloat:
    return "RtkFloat";
  case GnssFixType::RtkFixed:
    return "RtkFixed";
  case GnssFixType::PrecisePointPositioning:
    return "PrecisePointPositioning";
  }
  return "NoFix";
}

/// @brief Streams a GnssFixType as text.
inline std::ostream &operator<<(std::ostream &os, GnssFixType f) {
  return os << ToString(f);
}

// ── GnssConstellation
// ─────────────────────────────────────────────────────────

/// @brief Converts GnssConstellation to a stable diagnostic string.
constexpr std::string_view ToString(GnssConstellation c) noexcept {
  switch (c) {
  case GnssConstellation::Gps:
    return "GPS";
  case GnssConstellation::Glonass:
    return "GLONASS";
  case GnssConstellation::Galileo:
    return "Galileo";
  case GnssConstellation::BeiDou:
    return "BeiDou";
  case GnssConstellation::Qzss:
    return "QZSS";
  case GnssConstellation::Sbas:
    return "SBAS";
  case GnssConstellation::Unknown:
    return "Unknown";
  }
  return "Unknown";
}

/// @brief Streams a GnssConstellation as text.
inline std::ostream &operator<<(std::ostream &os, GnssConstellation c) {
  return os << ToString(c);
}

// ── EstimatorBackend
// ──────────────────────────────────────────────────────────

/// @brief Converts EstimatorBackend to a stable diagnostic string.
constexpr std::string_view ToString(estimation::EstimatorBackend b) noexcept {
  switch (b) {
  case estimation::EstimatorBackend::Unknown:
    return "Unknown";
  case estimation::EstimatorBackend::Ekf:
    return "EKF";
  case estimation::EstimatorBackend::CeresSlidingWindow:
    return "CeresSlidingWindow";
  case estimation::EstimatorBackend::GtsamFactorGraph:
    return "GtsamFactorGraph";
  case estimation::EstimatorBackend::Custom:
    return "Custom";
  }
  return "Unknown";
}

/// @brief Streams an EstimatorBackend as text.
inline std::ostream &operator<<(std::ostream &os,
                                estimation::EstimatorBackend b) {
  return os << ToString(b);
}

// ── EstimatorUpdateResult
// ─────────────────────────────────────────────────────

/// @brief Converts EstimatorUpdateResult to a stable diagnostic string.
constexpr std::string_view
ToString(estimation::EstimatorUpdateResult r) noexcept {
  switch (r) {
  case estimation::EstimatorUpdateResult::Accepted:
    return "Accepted";
  case estimation::EstimatorUpdateResult::Buffered:
    return "Buffered";
  case estimation::EstimatorUpdateResult::Rejected:
    return "Rejected";
  case estimation::EstimatorUpdateResult::OutOfOrder:
    return "OutOfOrder";
  case estimation::EstimatorUpdateResult::NotInitialized:
    return "NotInitialized";
  case estimation::EstimatorUpdateResult::BackendError:
    return "BackendError";
  }
  return "Unknown";
}

/// @brief Streams an EstimatorUpdateResult as text.
inline std::ostream &operator<<(std::ostream &os,
                                estimation::EstimatorUpdateResult r) {
  return os << ToString(r);
}

// ── AidingSource
// ──────────────────────────────────────────────────────────────

/// @brief Converts AidingSource to a stable diagnostic string.
constexpr std::string_view ToString(AidingSource s) noexcept {
  switch (s) {
  case AidingSource::Unknown:
    return "Unknown";
  case AidingSource::VisualOdometry:
    return "VisualOdometry";
  case AidingSource::GeoReference:
    return "GeoReference";
  case AidingSource::TerrainContourMatching:
    return "TERCOM";
  case AidingSource::ExternalSlam:
    return "ExternalSLAM";
  case AidingSource::SceneMatching:
    return "SceneMatching";
  case AidingSource::FeatureBasedNavigation:
    return "FeatureBasedNav";
  case AidingSource::RadioNavigation:
    return "RadioNavigation";
  }
  return "Unknown";
}

/// @brief Streams an AidingSource as text.
inline std::ostream &operator<<(std::ostream &os, AidingSource s) {
  return os << ToString(s);
}

} // namespace falconguide::core
