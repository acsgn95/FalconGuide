#pragma once

namespace falconguide::vo {

/// @brief Pinhole camera intrinsics.
struct CameraConfig {
    double fx{1000.0};  ///< Focal length x (pixels).
    double fy{1000.0};  ///< Focal length y (pixels).
    double cx{540.0};   ///< Principal point x.
    double cy{960.0};   ///< Principal point y.
    int width{1080};    ///< Image width.
    int height{1920};   ///< Image height.
};

}  // namespace falconguide::vo
