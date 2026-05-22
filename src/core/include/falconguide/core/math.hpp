#pragma once

/**
 * @file math.hpp
 * @brief Small math utilities used by navigation and estimation backends.
 */

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>

namespace falconguide::core {

/// @brief Pi constant used for angle conversion.
inline constexpr double kPi = 3.141592653589793238462643383279502884;

/// @brief Roll, pitch, and yaw angles in radians.
struct EulerAngles {
  double roll_rad{0.0};  ///< Rotation about the x-axis.
  double pitch_rad{0.0}; ///< Rotation about the y-axis.
  double yaw_rad{0.0};   ///< Rotation about the z-axis.
};

/// @brief Converts degrees to radians.
[[nodiscard]] inline double DegToRad(double degrees) {
  return degrees * kPi / 180.0;
}

/// @brief Converts radians to degrees.
[[nodiscard]] inline double RadToDeg(double radians) {
  return radians * 180.0 / kPi;
}

/// @brief Tests whether two scalar values are within an absolute tolerance.
[[nodiscard]] inline bool Near(double lhs, double rhs, double tolerance) {
  return std::abs(lhs - rhs) <= tolerance;
}

/// @brief Clamps a value to the inclusive range [min_value, max_value].
template <typename T>[[nodiscard]] T Clamp(T value, T min_value, T max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

/// @brief Wraps an angle to the [-pi, pi] interval.
[[nodiscard]] inline double WrapAngleRad(double angle_rad) {
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

/// @brief Builds the skew-symmetric matrix associated with a cross product.
[[nodiscard]] inline Eigen::Matrix3d
SkewSymmetric(const Eigen::Vector3d &vector) {
  Eigen::Matrix3d skew;
  skew << 0.0, -vector.z(), vector.y(), vector.z(), 0.0, -vector.x(),
      -vector.y(), vector.x(), 0.0;
  return skew;
}

/// @brief Normalizes a quaternion, returning identity for a zero-norm input.
[[nodiscard]] inline Eigen::Quaterniond
NormalizeQuaternion(const Eigen::Quaterniond &quaternion) {
  if (quaternion.norm() == 0.0) {
    return Eigen::Quaterniond::Identity();
  }
  return quaternion.normalized();
}

/// @brief Maps a rotation vector in so(3) to a unit quaternion in SO(3).
[[nodiscard]] inline Eigen::Quaterniond
ExpMapSo3(const Eigen::Vector3d &rotation_vector_rad) {
  const double angle = rotation_vector_rad.norm();
  if (angle < 1e-12) {
    return NormalizeQuaternion(Eigen::Quaterniond(
        1.0, 0.5 * rotation_vector_rad.x(), 0.5 * rotation_vector_rad.y(),
        0.5 * rotation_vector_rad.z()));
  }
  return Eigen::Quaterniond(
      Eigen::AngleAxisd(angle, rotation_vector_rad / angle));
}

/// @brief Maps a quaternion in SO(3) to its rotation-vector representation.
[[nodiscard]] inline Eigen::Vector3d
LogMapSo3(const Eigen::Quaterniond &quaternion) {
  const Eigen::Quaterniond normalized = NormalizeQuaternion(quaternion);
  const Eigen::AngleAxisd angle_axis(normalized);
  return angle_axis.angle() * angle_axis.axis();
}

/// @brief Converts roll-pitch-yaw angles to a yaw-pitch-roll quaternion.
[[nodiscard]] inline Eigen::Quaterniond
EulerToQuaternion(const EulerAngles &euler) {
  const Eigen::AngleAxisd roll(euler.roll_rad, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch(euler.pitch_rad, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw(euler.yaw_rad, Eigen::Vector3d::UnitZ());
  return NormalizeQuaternion(Eigen::Quaterniond(yaw * pitch * roll));
}

/// @brief Converts a quaternion to roll, pitch, and yaw angles.
[[nodiscard]] inline EulerAngles
QuaternionToEuler(const Eigen::Quaterniond &quaternion) {
  const Eigen::Matrix3d rotation =
      NormalizeQuaternion(quaternion).toRotationMatrix();
  const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
  const double pitch = std::asin(Clamp(-rotation(2, 0), -1.0, 1.0));
  const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  return EulerAngles{roll, pitch, yaw};
}

/// @brief Enforces covariance symmetry by averaging a matrix with its
/// transpose.
template <typename Matrix>
[[nodiscard]] Matrix SymmetrizeCovariance(const Matrix &covariance) {
  return 0.5 * (covariance + covariance.transpose());
}

} // namespace falconguide::core
