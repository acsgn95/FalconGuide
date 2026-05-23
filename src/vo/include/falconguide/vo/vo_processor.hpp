#pragma once

/**
 * @file vo_processor.hpp
 * @brief Frame-to-frame Visual Odometry using ORB features + Essential matrix.
 *
 * Pipeline per frame:
 *   1. Detect ORB keypoints
 *   2. Match against previous frame (BFMatcher + ratio test)
 *   3. Estimate Essential matrix (RANSAC)
 *   4. Recover R, t → accumulate global pose
 */

#include "falconguide/vo/camera_config.hpp"

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include <string>
#include <vector>

namespace falconguide::vo {

struct VoFrame {
    cv::Mat image;                        ///< Grayscale image.
    std::vector<cv::KeyPoint> keypoints;  ///< Detected keypoints.
    cv::Mat descriptors;                  ///< ORB descriptors.
    std::string path;                     ///< Source file path.
};

struct VoPose {
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);    ///< Rotation (world←camera).
    cv::Mat t = cv::Mat::zeros(3, 1, CV_64F);  ///< Translation (world frame, scale-unknown).
};

struct VoResult {
    VoFrame frame;                    ///< Current processed frame.
    VoPose pose;                      ///< Accumulated pose after this frame.
    std::vector<cv::DMatch> matches;  ///< Inlier matches (prev → current).
    bool initialized{false};          ///< False for the very first frame.
    int num_inliers{0};
};

class VoProcessor {
   public:
    explicit VoProcessor(CameraConfig cam = {});

    /// @brief Process a new camera frame.  Thread-safe (one-at-a-time).
    VoResult Process(const std::string& image_path);

    /// @brief Reset accumulated pose and frame history.
    void Reset();

    [[nodiscard]] const CameraConfig& Config() const { return cam_; }
    [[nodiscard]] const VoPose& CurrentPose() const { return pose_; }
    [[nodiscard]] const std::vector<VoPose>& Trajectory() const { return trajectory_; }

   private:
    CameraConfig cam_;
    cv::Mat K_;  ///< Camera intrinsic matrix.

    cv::Ptr<cv::ORB> detector_;
    cv::Ptr<cv::BFMatcher> matcher_;

    VoFrame prev_frame_;
    VoPose pose_;
    bool initialized_{false};

    std::vector<VoPose> trajectory_;
};

}  // namespace falconguide::vo
