#pragma once

/**
 * @file frames.hpp
 * @brief Strongly typed coordinate-frame tags, vectors, and rigid transforms.
 *
 * This header prevents accidental mixing of ECEF, ENU, body, and sensor-frame
 * vectors by encoding the frame in the C++ type system.
 */

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <type_traits>

namespace falconguide::core {

/// @brief Earth-centered, Earth-fixed WGS84 frame.
struct EcefFrame {};
/// @brief Local East-North-Up tangent-plane frame.
struct EnuFrame {};
/// @brief Local North-East-Down tangent-plane frame.
struct NedFrame {};
/// @brief Vehicle body frame.
struct BodyFrame {};
/// @brief IMU sensor frame.
struct ImuFrame {};
/// @brief Magnetometer sensor frame.
struct MagnetometerFrame {};
/// @brief Radar altimeter sensor frame.
struct RadarAltimeterFrame {};
/// @brief Generic range-finder sensor frame.
struct RangeFinderFrame {};
/// @brief Optical-flow sensor frame.
struct OpticalFlowFrame {};
/// @brief Doppler velocity log sensor frame.
struct DvlFrame {};
/// @brief Echo sounder sensor frame.
struct EchoSounderFrame {};
/// @brief Camera optical frame.
struct CameraFrame {};
/// @brief GNSS antenna phase-center frame.
struct GnssAntennaFrame {};
/// @brief Star-tracker sensor frame.
struct StarTrackerFrame {};

/**
 * @brief Three-dimensional vector tagged with a coordinate frame.
 * @tparam Frame Frame tag describing the coordinates stored in the vector.
 */
template <typename Frame> class Vec3 {
public:
  /// @brief Frame tag associated with this vector type.
  using FrameTag = Frame;

  /// @brief Creates a zero vector.
  constexpr Vec3() = default;
  /// @brief Creates a vector from an Eigen value.
  /// @param value Vector components in the frame identified by @p Frame.
  explicit Vec3(const Eigen::Vector3d &value) : value_(value) {}
  /// @brief Creates a vector from explicit x, y, and z components.
  Vec3(double x, double y, double z) : value_(x, y, z) {}

  /// @brief Returns the underlying Eigen vector.
  [[nodiscard]] const Eigen::Vector3d &eigen() const { return value_; }
  /// @brief Returns a mutable reference to the underlying Eigen vector.
  [[nodiscard]] Eigen::Vector3d &eigen() { return value_; }

  /// @brief Returns the x component.
  [[nodiscard]] double x() const { return value_.x(); }
  /// @brief Returns the y component.
  [[nodiscard]] double y() const { return value_.y(); }
  /// @brief Returns the z component.
  [[nodiscard]] double z() const { return value_.z(); }
  /// @brief Returns the Euclidean norm.
  [[nodiscard]] double norm() const { return value_.norm(); }

  /// @brief Adds two vectors in the same frame.
  [[nodiscard]] Vec3 operator+(const Vec3 &other) const {
    return Vec3(value_ + other.value_);
  }
  /// @brief Subtracts two vectors in the same frame.
  [[nodiscard]] Vec3 operator-(const Vec3 &other) const {
    return Vec3(value_ - other.value_);
  }
  /// @brief Scales the vector by a scalar.
  [[nodiscard]] Vec3 operator*(double scalar) const {
    return Vec3(value_ * scalar);
  }
  /// @brief Divides the vector by a scalar.
  [[nodiscard]] Vec3 operator/(double scalar) const {
    return Vec3(value_ / scalar);
  }

  /// @brief Adds another same-frame vector in place.
  Vec3 &operator+=(const Vec3 &other) {
    value_ += other.value_;
    return *this;
  }

  /// @brief Subtracts another same-frame vector in place.
  Vec3 &operator-=(const Vec3 &other) {
    value_ -= other.value_;
    return *this;
  }

private:
  Eigen::Vector3d value_{Eigen::Vector3d::Zero()};
};

/// @brief Left-hand scalar multiplication for a frame-tagged vector.
template <typename Frame>
[[nodiscard]] Vec3<Frame> operator*(double scalar, const Vec3<Frame> &vector) {
  return vector * scalar;
}

/**
 * @brief Rigid transform from @p FromFrame coordinates into @p ToFrame
 * coordinates.
 * @tparam FromFrame Source coordinate frame.
 * @tparam ToFrame Destination coordinate frame.
 */
template <typename FromFrame, typename ToFrame> class Transform {
public:
  /// @brief Source frame tag.
  using From = FromFrame;
  /// @brief Destination frame tag.
  using To = ToFrame;

  /// @brief Creates the identity transform.
  Transform() = default;
  /// @brief Creates a transform from rotation and translation.
  /// @param rotation_to_from Rotation applied to source vectors before
  /// translation.
  /// @param translation_to_from Translation expressed in the destination frame.
  Transform(const Eigen::Quaterniond &rotation_to_from,
            const Vec3<ToFrame> &translation_to_from)
      : rotation_to_from_(rotation_to_from.normalized()),
        translation_to_from_(translation_to_from) {}

  /// @brief Returns an identity transform.
  [[nodiscard]] static Transform Identity() { return Transform(); }

  /// @brief Applies the transform to a point in the source frame.
  [[nodiscard]] Vec3<ToFrame>
  operator*(const Vec3<FromFrame> &point_from) const {
    return Vec3<ToFrame>(rotation_to_from_ * point_from.eigen() +
                         translation_to_from_.eigen());
  }

  /// @brief Returns the inverse transform.
  [[nodiscard]] Transform<ToFrame, FromFrame> inverse() const {
    const Eigen::Quaterniond inverse_rotation = rotation_to_from_.conjugate();
    const Eigen::Vector3d inverse_translation =
        -(inverse_rotation * translation_to_from_.eigen());
    return Transform<ToFrame, FromFrame>(inverse_rotation,
                                         Vec3<FromFrame>(inverse_translation));
  }

  /// @brief Returns the rotation component.
  [[nodiscard]] const Eigen::Quaterniond &rotation() const {
    return rotation_to_from_;
  }
  /// @brief Returns the translation component.
  [[nodiscard]] const Vec3<ToFrame> &translation() const {
    return translation_to_from_;
  }

private:
  Eigen::Quaterniond rotation_to_from_{Eigen::Quaterniond::Identity()};
  Vec3<ToFrame> translation_to_from_{};
};

/// @brief Composes two frame-compatible transforms.
template <typename LeftFrom, typename SharedFrame, typename RightTo>
[[nodiscard]] Transform<LeftFrom, RightTo>
operator*(const Transform<SharedFrame, RightTo> &left,
          const Transform<LeftFrom, SharedFrame> &right) {
  const Eigen::Quaterniond rotation = left.rotation() * right.rotation();
  const Vec3<RightTo> translation(left.rotation() *
                                      right.translation().eigen() +
                                  left.translation().eigen());
  return Transform<LeftFrom, RightTo>(rotation, translation);
}

/// @brief Type trait that is true for Vec3 specializations.
template <typename T> inline constexpr bool IsFrameVector = false;

/// @brief Vec3 specialization of IsFrameVector.
template <typename Frame>
inline constexpr bool IsFrameVector<Vec3<Frame>> = true;

} // namespace falconguide::core
