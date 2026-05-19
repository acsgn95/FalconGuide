#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <type_traits>

namespace falconguide::core {

struct EcefFrame {};
struct EnuFrame {};
struct NedFrame {};
struct BodyFrame {};
struct ImuFrame {};
struct MagnetometerFrame {};
struct RadarAltimeterFrame {};
struct RangeFinderFrame {};
struct OpticalFlowFrame {};
struct DvlFrame {};
struct EchoSounderFrame {};
struct CameraFrame {};
struct GnssAntennaFrame {};
struct StarTrackerFrame {};

template <typename Frame>
class Vec3 {
 public:
  using FrameTag = Frame;

  constexpr Vec3() = default;
  explicit Vec3(const Eigen::Vector3d& value) : value_(value) {}
  Vec3(double x, double y, double z) : value_(x, y, z) {}

  [[nodiscard]] const Eigen::Vector3d& eigen() const { return value_; }
  [[nodiscard]] Eigen::Vector3d& eigen() { return value_; }

  [[nodiscard]] double x() const { return value_.x(); }
  [[nodiscard]] double y() const { return value_.y(); }
  [[nodiscard]] double z() const { return value_.z(); }
  [[nodiscard]] double norm() const { return value_.norm(); }

  [[nodiscard]] Vec3 operator+(const Vec3& other) const { return Vec3(value_ + other.value_); }
  [[nodiscard]] Vec3 operator-(const Vec3& other) const { return Vec3(value_ - other.value_); }
  [[nodiscard]] Vec3 operator*(double scalar) const { return Vec3(value_ * scalar); }
  [[nodiscard]] Vec3 operator/(double scalar) const { return Vec3(value_ / scalar); }

  Vec3& operator+=(const Vec3& other) {
    value_ += other.value_;
    return *this;
  }

  Vec3& operator-=(const Vec3& other) {
    value_ -= other.value_;
    return *this;
  }

 private:
  Eigen::Vector3d value_{Eigen::Vector3d::Zero()};
};

template <typename Frame>
[[nodiscard]] Vec3<Frame> operator*(double scalar, const Vec3<Frame>& vector) {
  return vector * scalar;
}

template <typename FromFrame, typename ToFrame>
class Transform {
 public:
  using From = FromFrame;
  using To = ToFrame;

  Transform() = default;
  Transform(const Eigen::Quaterniond& rotation_to_from, const Vec3<ToFrame>& translation_to_from)
      : rotation_to_from_(rotation_to_from.normalized()), translation_to_from_(translation_to_from) {}

  [[nodiscard]] static Transform Identity() { return Transform(); }

  [[nodiscard]] Vec3<ToFrame> operator*(const Vec3<FromFrame>& point_from) const {
    return Vec3<ToFrame>(rotation_to_from_ * point_from.eigen() + translation_to_from_.eigen());
  }

  [[nodiscard]] Transform<ToFrame, FromFrame> inverse() const {
    const Eigen::Quaterniond inverse_rotation = rotation_to_from_.conjugate();
    const Eigen::Vector3d inverse_translation = -(inverse_rotation * translation_to_from_.eigen());
    return Transform<ToFrame, FromFrame>(inverse_rotation, Vec3<FromFrame>(inverse_translation));
  }

  [[nodiscard]] const Eigen::Quaterniond& rotation() const { return rotation_to_from_; }
  [[nodiscard]] const Vec3<ToFrame>& translation() const { return translation_to_from_; }

 private:
  Eigen::Quaterniond rotation_to_from_{Eigen::Quaterniond::Identity()};
  Vec3<ToFrame> translation_to_from_{};
};

template <typename LeftFrom, typename SharedFrame, typename RightTo>
[[nodiscard]] Transform<LeftFrom, RightTo> operator*(
    const Transform<SharedFrame, RightTo>& left,
    const Transform<LeftFrom, SharedFrame>& right) {
  const Eigen::Quaterniond rotation = left.rotation() * right.rotation();
  const Vec3<RightTo> translation(left.rotation() * right.translation().eigen() + left.translation().eigen());
  return Transform<LeftFrom, RightTo>(rotation, translation);
}

template <typename T>
inline constexpr bool IsFrameVector = false;

template <typename Frame>
inline constexpr bool IsFrameVector<Vec3<Frame>> = true;

}  // namespace falconguide::core
