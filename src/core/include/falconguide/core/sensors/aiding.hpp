#pragma once

#include "falconguide/core/frames.hpp"
#include "falconguide/core/sensors/common.hpp"
#include "falconguide/core/time.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <optional>

namespace falconguide::core {

// ── Aiding Source ─────────────────────────────────────────────────────────────
//
// Identifies the algorithm or sensor pipeline that produced an AidingSolution.
// The estimator uses this to apply the correct noise model and weighting.
//
enum class AidingSource {
  Unknown,
  VisualOdometry,              // PnP from camera frames (forward/stereo)
  GeoReference,                // camera (downward) + satellite/map image matching + DEM → 3D pos + yaw
  TerrainContourMatching,      // TERCOM: radar altimeter profile + DEM → 3D pos
  ExternalSlam,                // external SLAM system (LiDAR, visual, etc.)
  SceneMatching,               // optical scene matching against pre-built mosaic
  FeatureBasedNavigation,      // landmark / known feature matching
  RadioNavigation,             // DME, VOR, TACAN, LORAN — range/bearing based
};

// ── Orientation Validity Mask ─────────────────────────────────────────────────
//
// Most aiding sources do not provide full 6-DOF orientation.
// This mask tells the estimator which attitude components are trustworthy.
//
struct OrientationValidity {
  bool yaw{false};    // heading — GeoRef, SceneMatch can provide this
  bool pitch{false};  // PnP VO with sufficient baseline can provide this
  bool roll{false};   // rarely from aiding, usually from IMU/magnetometer

  [[nodiscard]] bool any()  const { return yaw || pitch || roll; }
  [[nodiscard]] bool full() const { return yaw && pitch && roll; }
};

// ── AidingSolution ────────────────────────────────────────────────────────────
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
//   match_score             — algorithm-specific quality [0, 1], higher = better
//                             Used for adaptive weighting / outlier rejection.
//
struct AidingSolution {
  Timestamp   timestamp;
  AidingSource source{AidingSource::Unknown};

  // ── Position ───────────────────────────────────────────────────────────────
  // TERCOM, GeoRef, VO, SLAM: typically all three axes.
  // RadioNav: may only have horizontal (set z covariance large if altitude unknown).
  std::optional<Vec3<EcefFrame>> position_ecef_m;
  Eigen::Matrix3d position_covariance_ecef_m2{Eigen::Matrix3d::Zero()};

  // ── Velocity ───────────────────────────────────────────────────────────────
  // VO can provide velocity from consecutive frame delta.
  std::optional<Vec3<EcefFrame>> velocity_ecef_mps;
  Eigen::Matrix3d velocity_covariance_ecef_mps2{Eigen::Matrix3d::Zero()};

  // ── Orientation ────────────────────────────────────────────────────────────
  // GeoRef typically provides yaw only; VO can provide full attitude.
  // Always check orientation_validity before using individual axes.
  std::optional<Eigen::Quaterniond> orientation_body_to_ecef;
  OrientationValidity orientation_validity;
  // Per-axis attitude uncertainty (rad), indexed [roll, pitch, yaw].
  Eigen::Vector3d orientation_sigma_rad{Eigen::Vector3d::Ones() * 1e6};

  // ── Quality ────────────────────────────────────────────────────────────────
  // Normalised match confidence [0, 1]. Estimator may reject if below threshold.
  std::optional<double> match_score;

  MeasurementValidity validity{MeasurementValidity::Valid};
};

}  // namespace falconguide::core
