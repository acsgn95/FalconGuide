#pragma once

/**
 * @file aiding.hpp
 * @brief Unified aided-navigation solution types from vision, terrain, SLAM,
 * and radio sources.
 */

#include "falconguide/core/frames.hpp"
#include "falconguide/core/sensors/common.hpp"
#include "falconguide/core/time.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <optional>

namespace falconguide::core {

// ── Aiding Source
// ─────────────────────────────────────────────────────────────
//
// Identifies the algorithm or sensor pipeline that produced an AidingSolution.
// The estimator uses this to apply the correct noise model and weighting.
//
enum class AidingSource {
  Unknown,                ///< Source is not known.
  VisualOdometry,         ///< PnP or visual odometry from camera frames.
  GeoReference,           ///< Camera plus map/satellite matching and DEM.
  TerrainContourMatching, ///< Terrain contour matching using altimetry and DEM.
  ExternalSlam,  ///< External SLAM system such as lidar or visual SLAM.
  SceneMatching, ///< Optical scene matching against a pre-built mosaic.
  FeatureBasedNavigation, ///< Landmark or known-feature matching.
  RadioNavigation,        ///< Range/bearing radio navigation source.
};

// ── Orientation Validity Mask
// ─────────────────────────────────────────────────
//
// Most aiding sources do not provide full 6-DOF orientation.
// This mask tells the estimator which attitude components are trustworthy.
//
struct OrientationValidity {
  bool yaw{false};   ///< True when yaw/heading is valid.
  bool pitch{false}; ///< True when pitch is valid.
  bool roll{false};  ///< True when roll is valid.

  /// @brief Returns true when at least one attitude component is valid.
  [[nodiscard]] bool any() const { return yaw || pitch || roll; }
  /// @brief Returns true when yaw, pitch, and roll are all valid.
  [[nodiscard]] bool full() const { return yaw && pitch && roll; }
};

// ── AidingSolution
// ────────────────────────────────────────────────────────────
//
// Unified output type for all terrain-aided, image-aided, and visual-odometry
// navigation algorithms. The estimator checks which optional fields are set
// and applies only the available observations as update steps.
//
// Field conventions:
//   position_ecef_m         — 3D position in ECEF (WGS84)
//   velocity_ecef_mps       — ECEF velocity (VO can provide this)
//   orientation_body_to_ecef — full quaternion, use orientation_validity to
//                              know which axes are meaningful
//   match_score             — algorithm-specific quality [0, 1], higher =
//   better
//                             Used for adaptive weighting / outlier rejection.
//
struct AidingSolution {
  Timestamp timestamp;                        ///< Solution timestamp.
  AidingSource source{AidingSource::Unknown}; ///< Algorithm or sensor pipeline
                                              ///< that produced the solution.

  // ── Position ───────────────────────────────────────────────────────────────
  // TERCOM, GeoRef, VO, SLAM: typically all three axes.
  // RadioNav: may only have horizontal (set z covariance large if altitude
  // unknown).
  std::optional<Vec3<EcefFrame>>
      position_ecef_m; ///< Optional ECEF position observation.
  Eigen::Matrix3d position_covariance_ecef_m2{
      Eigen::Matrix3d::Zero()}; ///< ECEF position covariance.

  // ── Velocity ───────────────────────────────────────────────────────────────
  // VO can provide velocity from consecutive frame delta.
  std::optional<Vec3<EcefFrame>>
      velocity_ecef_mps; ///< Optional ECEF velocity observation.
  Eigen::Matrix3d velocity_covariance_ecef_mps2{
      Eigen::Matrix3d::Zero()}; ///< ECEF velocity covariance.

  // ── Orientation ────────────────────────────────────────────────────────────
  // GeoRef typically provides yaw only; VO can provide full attitude.
  // Always check orientation_validity before using individual axes.
  std::optional<Eigen::Quaterniond>
      orientation_body_to_ecef; ///< Optional body-to-ECEF orientation.
  OrientationValidity
      orientation_validity; ///< Component-level attitude validity.
  // Per-axis attitude uncertainty (rad), indexed [roll, pitch, yaw].
  Eigen::Vector3d orientation_sigma_rad{Eigen::Vector3d::Ones() *
                                        1e6}; ///< Per-axis attitude sigma.

  // ── Quality ────────────────────────────────────────────────────────────────
  // Normalised match confidence [0, 1]. Estimator may reject if below
  // threshold.
  std::optional<double>
      match_score; ///< Optional normalized matching confidence.

  MeasurementValidity validity{
      MeasurementValidity::Valid}; ///< Source validity state.
};

} // namespace falconguide::core
