#pragma once

#include "falconguide/core/frames.hpp"
#include "falconguide/core/time.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <optional>

namespace falconguide::core {

enum class NavigationStatus {
  Unknown,
  NotInitialized,
  Initializing,
  Nominal,
  Degraded,
  DeadReckoning,
  Fault
};

enum class EstimatorMode {
  Unknown,
  InertialOnly,
  VisualInertial,
  GnssInertial,
  VisualInertialGnss,
  MultiSensorFusion
};

enum class SensorHealth {
  Unknown,
  Healthy,
  Degraded,
  Rejected,
  Missing,
  Stale,
  Fault
};

struct SensorStatus {
  SensorHealth health{SensorHealth::Unknown};
  bool used_in_solution{false};
  std::optional<double> innovation_norm;
  std::optional<double> last_update_age_s;
};

struct NavigationSensorStatus {
  SensorStatus imu;
  SensorStatus gnss;
  SensorStatus vision;
  SensorStatus magnetometer;
  SensorStatus barometer;
  SensorStatus radar_altimeter;
  SensorStatus range_finder;
  SensorStatus optical_flow;
  SensorStatus wheel_odometry;
  SensorStatus airspeed;
  SensorStatus dvl;
  SensorStatus echo_sounder;
  SensorStatus external_pose;
  SensorStatus external_velocity;
  SensorStatus external_odometry;
};

struct NavigationQuality {
  bool initialized{false};
  bool degraded{false};
  std::optional<double> position_accuracy_m;
  std::optional<double> velocity_accuracy_mps;
  std::optional<double> attitude_accuracy_rad;
  std::optional<double> horizontal_accuracy_m;
  std::optional<double> vertical_accuracy_m;
};

struct NavigationState {
  Timestamp timestamp;

  Vec3<EcefFrame> position_ecef_m;
  Vec3<EcefFrame> velocity_ecef_mps;
  Vec3<EnuFrame> position_enu_m;
  Vec3<EnuFrame> velocity_enu_mps;

  Eigen::Quaterniond orientation_body_to_enu{Eigen::Quaterniond::Identity()};
  Vec3<BodyFrame> angular_rate_body_radps;
  Vec3<BodyFrame> specific_force_body_mps2;

  Eigen::Matrix<double, 15, 15> covariance{Eigen::Matrix<double, 15, 15>::Zero()};

  NavigationStatus status{NavigationStatus::Unknown};
  EstimatorMode mode{EstimatorMode::Unknown};
  NavigationQuality quality;
  NavigationSensorStatus sensors;
};

}  // namespace falconguide::core
