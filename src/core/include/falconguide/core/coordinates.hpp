#pragma once

/**
 * @file coordinates.hpp
 * @brief WGS84 geodetic conversion and local tangent plane helpers.
 */

#include "falconguide/core/frames.hpp"

#include <Eigen/Core>

#include <cmath>

namespace falconguide::core {

/// @brief Geodetic latitude, longitude, and altitude on WGS84.
struct Lla {
    double latitude_rad{0.0};   ///< Geodetic latitude in radians.
    double longitude_rad{0.0};  ///< Geodetic longitude in radians.
    double altitude_m{0.0};     ///< Altitude above the WGS84 ellipsoid in meters.
};

/// @brief WGS84 ellipsoid constants.
namespace wgs84 {
/// @brief WGS84 semi-major axis in meters.
inline constexpr double kSemiMajorAxisM = 6378137.0;
/// @brief WGS84 flattening.
inline constexpr double kFlattening = 1.0 / 298.257223563;
/// @brief WGS84 semi-minor axis in meters.
inline constexpr double kSemiMinorAxisM = kSemiMajorAxisM * (1.0 - kFlattening);
/// @brief First eccentricity squared.
inline constexpr double kFirstEccentricitySquared = kFlattening * (2.0 - kFlattening);
/// @brief Second eccentricity squared.
inline constexpr double kSecondEccentricitySquared = kFirstEccentricitySquared / (1.0 - kFirstEccentricitySquared);
}  // namespace wgs84

/// @brief Converts WGS84 latitude/longitude/altitude to ECEF position.
/// @param lla Geodetic position.
/// @return ECEF position in meters.
[[nodiscard]] inline Vec3<EcefFrame> LlaToEcef(const Lla &lla) {
    const double sin_lat = std::sin(lla.latitude_rad);
    const double cos_lat = std::cos(lla.latitude_rad);
    const double sin_lon = std::sin(lla.longitude_rad);
    const double cos_lon = std::cos(lla.longitude_rad);
    const double prime_vertical_radius =
        wgs84::kSemiMajorAxisM / std::sqrt(1.0 - wgs84::kFirstEccentricitySquared * sin_lat * sin_lat);

    const double x = (prime_vertical_radius + lla.altitude_m) * cos_lat * cos_lon;
    const double y = (prime_vertical_radius + lla.altitude_m) * cos_lat * sin_lon;
    const double z = (prime_vertical_radius * (1.0 - wgs84::kFirstEccentricitySquared) + lla.altitude_m) * sin_lat;
    return Vec3<EcefFrame>(x, y, z);
}

/// @brief Converts an ECEF position to WGS84 latitude/longitude/altitude.
/// @param ecef ECEF position in meters.
/// @return Geodetic position on the WGS84 ellipsoid.
[[nodiscard]] inline Lla EcefToLla(const Vec3<EcefFrame> &ecef) {
    const double x = ecef.x();
    const double y = ecef.y();
    const double z = ecef.z();
    const double p = std::hypot(x, y);
    const double theta = std::atan2(z * wgs84::kSemiMajorAxisM, p * wgs84::kSemiMinorAxisM);
    const double sin_theta = std::sin(theta);
    const double cos_theta = std::cos(theta);

    const double latitude =
        std::atan2(z + wgs84::kSecondEccentricitySquared * wgs84::kSemiMinorAxisM * sin_theta * sin_theta * sin_theta,
                   p - wgs84::kFirstEccentricitySquared * wgs84::kSemiMajorAxisM * cos_theta * cos_theta * cos_theta);
    const double longitude = std::atan2(y, x);

    const double sin_lat = std::sin(latitude);
    const double prime_vertical_radius =
        wgs84::kSemiMajorAxisM / std::sqrt(1.0 - wgs84::kFirstEccentricitySquared * sin_lat * sin_lat);
    const double altitude = p / std::cos(latitude) - prime_vertical_radius;

    return Lla{latitude, longitude, altitude};
}

/// @brief Returns the rotation matrix that maps ECEF deltas into ENU
/// coordinates.
[[nodiscard]] inline Eigen::Matrix3d EcefToEnuRotation(double latitude_rad, double longitude_rad) {
    const double sin_lat = std::sin(latitude_rad);
    const double cos_lat = std::cos(latitude_rad);
    const double sin_lon = std::sin(longitude_rad);
    const double cos_lon = std::cos(longitude_rad);

    Eigen::Matrix3d rotation;
    rotation << -sin_lon, cos_lon, 0.0, -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat, cos_lat * cos_lon,
        cos_lat * sin_lon, sin_lat;
    return rotation;
}

/// @brief Local East-North-Up frame anchored at a fixed WGS84 origin.
class LocalTangentPlane {
   public:
    /// @brief Creates a tangent plane anchored at @p origin_lla.
    explicit LocalTangentPlane(const Lla &origin_lla)
        : origin_lla_(origin_lla),
          origin_ecef_m_(LlaToEcef(origin_lla)),
          ecef_to_enu_rotation_(EcefToEnuRotation(origin_lla.latitude_rad, origin_lla.longitude_rad)) {}

    /// @brief Returns the geodetic origin of the tangent plane.
    [[nodiscard]] const Lla &origin_lla() const { return origin_lla_; }
    /// @brief Returns the ECEF origin of the tangent plane.
    [[nodiscard]] const Vec3<EcefFrame> &origin_ecef_m() const { return origin_ecef_m_; }
    /// @brief Returns the ECEF-to-ENU rotation matrix at the origin.
    [[nodiscard]] const Eigen::Matrix3d &ecef_to_enu_rotation() const { return ecef_to_enu_rotation_; }

    /// @brief Converts an ECEF point to local ENU coordinates.
    [[nodiscard]] Vec3<EnuFrame> EcefToEnu(const Vec3<EcefFrame> &point_ecef_m) const {
        const Eigen::Vector3d delta_ecef = point_ecef_m.eigen() - origin_ecef_m_.eigen();
        return Vec3<EnuFrame>(ecef_to_enu_rotation_ * delta_ecef);
    }

    /// @brief Converts a local ENU point to ECEF coordinates.
    [[nodiscard]] Vec3<EcefFrame> EnuToEcef(const Vec3<EnuFrame> &point_enu_m) const {
        return Vec3<EcefFrame>(origin_ecef_m_.eigen() + ecef_to_enu_rotation_.transpose() * point_enu_m.eigen());
    }

   private:
    Lla origin_lla_;
    Vec3<EcefFrame> origin_ecef_m_;
    Eigen::Matrix3d ecef_to_enu_rotation_{Eigen::Matrix3d::Identity()};
};

}  // namespace falconguide::core
