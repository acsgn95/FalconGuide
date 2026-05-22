#pragma once

/**
 * @file external.hpp
 * @brief External pose, velocity, and odometry injection types.
 */

#include "falconguide/core/frames.hpp"
#include "falconguide/core/sensors/common.hpp"
#include "falconguide/core/time.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace falconguide::core {

/// @brief Source system that produced an external navigation measurement.
enum class ExternalMeasurementSource {
  Unknown,       ///< Source is not known.
  MotionCapture, ///< Motion-capture system.
  Fiducial,      ///< Fiducial marker tracker.
  Uwb,           ///< UWB localization system.
  ExternalSlam,  ///< External SLAM stack.
  VehicleCan,    ///< Vehicle CAN bus estimate.
  OperatorInput  ///< Manual or operator-supplied input.
};

/// @brief Timestamped external pose measurement.
struct ExternalPoseMeasurement {
  Timestamp timestamp; ///< Sample timestamp.
  ExternalMeasurementSource source{
      ExternalMeasurementSource::Unknown}; ///< Producer of the pose estimate.
  Vec3<EcefFrame> position_ecef_m;         ///< ECEF position in meters.
  Eigen::Quaterniond orientation_body_to_ecef{
      Eigen::Quaterniond::Identity()}; ///< Body-to-ECEF orientation.
  Eigen::Matrix<double, 6, 6> covariance{
      Eigen::Matrix<double, 6, 6>::Zero()}; ///< Pose covariance.
  MeasurementValidity validity{
      MeasurementValidity::Valid}; ///< Source validity state.
};

/// @brief Timestamped external body-frame velocity measurement.
struct ExternalVelocityMeasurement {
  Timestamp timestamp; ///< Sample timestamp.
  ExternalMeasurementSource source{
      ExternalMeasurementSource::Unknown};  ///< Producer of the velocity
                                            ///< estimate.
  Vec3<BodyFrame> linear_velocity_body_mps; ///< Body-frame linear velocity.
  Vec3<BodyFrame> angular_rate_body_radps;  ///< Body-frame angular rate.
  Eigen::Matrix<double, 6, 6> covariance{
      Eigen::Matrix<double, 6, 6>::Zero()}; ///< Velocity covariance.
  MeasurementValidity validity{
      MeasurementValidity::Valid}; ///< Source validity state.
};

/// @brief Timestamped external position, velocity, and attitude measurement.
struct ExternalOdometryMeasurement {
  Timestamp timestamp; ///< Sample timestamp.
  ExternalMeasurementSource source{
      ExternalMeasurementSource::Unknown}; ///< Producer of the odometry
                                           ///< estimate.
  Vec3<EcefFrame> position_ecef_m;         ///< ECEF position in meters.
  Vec3<EcefFrame> velocity_ecef_mps; ///< ECEF velocity in meters per second.
  Eigen::Quaterniond orientation_body_to_ecef{
      Eigen::Quaterniond::Identity()}; ///< Body-to-ECEF orientation.
  Eigen::Matrix<double, 9, 9> covariance{
      Eigen::Matrix<double, 9, 9>::Zero()}; ///< Pose and velocity covariance.
  MeasurementValidity validity{
      MeasurementValidity::Valid}; ///< Source validity state.
};

} // namespace falconguide::core
