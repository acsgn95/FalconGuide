#pragma once

/**
 * @file camera.hpp
 * @brief Camera metadata and image-aiding measurement types.
 */

#include "falconguide/core/frames.hpp"
#include "falconguide/core/sensors/common.hpp"
#include "falconguide/core/time.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <optional>
#include <string>

namespace falconguide::core {

/// @brief Camera intrinsic model parameters.
struct CameraIntrinsics {
    std::uint32_t width{0};      ///< Image width in pixels.
    std::uint32_t height{0};     ///< Image height in pixels.
    Eigen::VectorXd parameters;  ///< Model-specific intrinsic parameters.
    std::string model;           ///< Intrinsic model name, e.g. pinhole or fisheye.
};

/// @brief Intended role of a camera stream.
enum class CameraRole {
    Unknown,         ///< Role is not known.
    VisualOdometry,  ///< Camera used for visual odometry.
    GeoReference,    ///< Camera used for map or satellite image matching.
    Mapping,         ///< Camera used for mapping.
    Landing,         ///< Camera used for landing guidance.
    Inspection       ///< Camera used for inspection.
};

/// @brief Camera calibration block.
struct CameraCalibration {
    std::string camera_name;                           ///< Stable camera identifier.
    CameraRole role{CameraRole::Unknown};              ///< Intended camera role.
    CameraIntrinsics intrinsics;                       ///< Intrinsic calibration.
    Transform<CameraFrame, BodyFrame> camera_to_body;  ///< Camera to body transform.
};

/// @brief Runtime configuration for a camera stream.
struct CameraConfig {
    std::string camera_name;               ///< Stable camera identifier.
    CameraRole role{CameraRole::Unknown};  ///< Intended camera role.
    double nominal_frame_rate_hz{0.0};     ///< Expected frame rate.
    bool hardware_synchronized{false};     ///< True when camera timestamps are hardware synchronized.
};

/// @brief Timestamped camera frame metadata.
struct CameraFrameMeasurement {
    Timestamp timestamp;                                       ///< Frame timestamp.
    std::uint64_t frame_id{0};                                 ///< Monotonic frame identifier.
    std::string camera_name;                                   ///< Camera that produced the frame.
    CameraRole role{CameraRole::Unknown};                      ///< Role of the camera stream.
    std::optional<double> exposure_time_s;                     ///< Optional exposure time.
    std::optional<double> gain;                                ///< Optional sensor gain.
    MeasurementValidity validity{MeasurementValidity::Valid};  ///< Source validity state.
};

/// @brief Metadata for an image prepared for georeference matching.
struct GeoReferenceImageMeasurement {
    Timestamp timestamp;                                         ///< Frame timestamp.
    std::uint64_t frame_id{0};                                   ///< Source frame identifier.
    std::string camera_name;                                     ///< Camera that produced the frame.
    std::optional<Vec3<EcefFrame>> approximate_position_ecef_m;  ///< Optional approximate search position.
    std::optional<double> ground_sample_distance_m;              ///< Optional ground sample distance.
    std::optional<double> yaw_prior_rad;                         ///< Optional yaw prior for matching.
    MeasurementValidity validity{MeasurementValidity::Valid};    ///< Source validity state.
};

}  // namespace falconguide::core
