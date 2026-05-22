#pragma once

#include "falconguide/core/frames.hpp"
#include "falconguide/core/sensors/common.hpp"
#include "falconguide/core/time.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <optional>

namespace falconguide::core {

// ── StarTrackerMeasurement ────────────────────────────────────────────────────
//
// Celestial navigation fix derived from star field observation.
//
// Primary observables are geodetic latitude, longitude, and true heading (yaw).
// Pitch and roll require a known local gravity vector or horizon reference and
// are therefore geometrically dependent; include them when the processing
// pipeline can produce a reliable estimate, leave them empty otherwise.
//
// All angles are in radians. Sensor drivers that output degrees must convert
// before filling this struct.
//
struct StarTrackerMeasurement {
  Timestamp timestamp;

  // ── Geodetic position ───────────────────────────────────────────────────────
  // Latitude and longitude derived from star sightings + UTC time.
  // Altitude is not observable by the star tracker — set it externally if needed.
  double latitude_rad{0.0};
  double longitude_rad{0.0};

  // 1-sigma horizontal position uncertainty (rad, great-circle).
  double position_sigma_rad{0.0};

  // ── Heading ────────────────────────────────────────────────────────────────
  // True heading (yaw) relative to geographic north, NED convention.
  double yaw_rad{0.0};
  double yaw_sigma_rad{0.0};

  // ── Pitch and roll (optional) ──────────────────────────────────────────────
  // Available only when the pipeline couples the star fix with a horizon or
  // gravity reference. Leave empty when unavailable.
  std::optional<double> pitch_rad;
  std::optional<double> roll_rad;
  // 1-sigma [pitch, roll] uncertainty (rad). Ignored when the fields above are empty.
  Eigen::Vector2d pitch_roll_sigma_rad{Eigen::Vector2d::Ones() * 1e6};

  // ── Quality ────────────────────────────────────────────────────────────────
  std::optional<std::uint16_t> stars_tracked;
  // Lost-in-space confidence [0, 1]: 1 = catalog match fully converged.
  std::optional<double> lost_in_space_confidence;

  MeasurementValidity validity{MeasurementValidity::Valid};
};

// ── StarTrackerCalibration ────────────────────────────────────────────────────
struct StarTrackerCalibration {
  Transform<StarTrackerFrame, BodyFrame> star_tracker_to_body;
  double field_of_view_rad{0.0};
};

}  // namespace falconguide::core
