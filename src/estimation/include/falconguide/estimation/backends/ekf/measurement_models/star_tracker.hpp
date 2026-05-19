#pragma once

#include "falconguide/estimation/backends/ekf/measurement_models/measurement_model.hpp"

#include <Eigen/Geometry>
#include <optional>

namespace falconguide::estimation::ekf {

struct StarTrackerOptions {
  // Rotation from star-tracker sensor frame to body frame.
  Eigen::Quaterniond star_tracker_to_body{Eigen::Quaterniond::Identity()};

  // Attitude noise (rad).
  double sigma_yaw_rad{0.001};
  double sigma_pitch_rad{0.001};
  double sigma_roll_rad{0.001};

  // Use pitch and roll components from measurement (requires pitch_rad / roll_rad present).
  bool use_pitch_roll{true};

  // Mahalanobis gate (chi-squared, 1 or 3-DOF).  0 = disabled.
  double innovation_gate{0.0};
};

// ── StarTracker ───────────────────────────────────────────────────────────────
//
// Attitude update from a celestial star tracker.
//
// The star tracker reports the boresight direction (lon/lat in celestial
// coordinates) plus an in-plane rotation (yaw), defining the full body
// attitude in an inertial-like frame.  For short-duration navigation this is
// treated as a direct body-to-ENU attitude measurement.
//
// Quaternion construction from (lon, lat, yaw):
//   q = Rz(lon) * Ry(π/2 − lat) * Rz(−yaw) * q_star_tracker_to_body
//
// Measurement function:
//   h(x) = q_body_to_enu  (attitude quaternion)
//
// Innovation:
//   δθ = LogMapSo3(q_pred^{−1} ⊗ q_obs)
//
// H: I₃ at attitude block (full or yaw-only depending on options).
//
class StarTracker : public IMeasurementModel {
 public:
  explicit StarTracker(StarTrackerOptions options = {});

  [[nodiscard]] bool CanHandle(const SensorMeasurement& measurement) const override;

  EstimatorUpdateResult Apply(
      NominalState& nominal,
      Eigen::VectorXd& error_state,
      Eigen::MatrixXd& covariance,
      const StateLayout& layout,
      const SensorMeasurement& measurement,
      const UpdateContext& context) override;

 private:
  StarTrackerOptions options_;
};

}  // namespace falconguide::estimation::ekf
