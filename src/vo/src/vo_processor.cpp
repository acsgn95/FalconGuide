#include "falconguide/vo/vo_processor.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace falconguide::vo {

VoProcessor::VoProcessor(CameraConfig cam) : cam_(cam) {
    K_ = (cv::Mat_<double>(3, 3) << cam_.fx, 0, cam_.cx, 0, cam_.fy, cam_.cy, 0, 0, 1);

    detector_ = cv::ORB::create(2000,  // nfeatures
                                1.2f,  // scaleFactor
                                8,     // nlevels
                                31,    // edgeThreshold
                                0,     // firstLevel
                                2,     // WTA_K
                                cv::ORB::HARRIS_SCORE,
                                31,  // patchSize
                                20   // fastThreshold
    );

    matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING, false);
}

void VoProcessor::Reset() {
    prev_frame_ = {};
    pose_ = {};
    initialized_ = false;
    trajectory_.clear();
}

VoResult VoProcessor::Process(const std::string& image_path) {
    VoResult res;
    res.pose = pose_;

    cv::Mat color = cv::imread(image_path, cv::IMREAD_COLOR);
    if (color.empty()) return res;

    cv::Mat gray;
    cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);

    VoFrame cur;
    cur.path = image_path;
    cur.image = color;
    detector_->detectAndCompute(gray, cv::noArray(), cur.keypoints, cur.descriptors);
    res.frame = cur;

    if (!initialized_ || prev_frame_.descriptors.empty()) {
        prev_frame_ = cur;
        initialized_ = true;
        res.initialized = false;
        trajectory_.push_back(pose_);
        return res;
    }

    // ── Match ──────────────────────────────────────────────────────────────
    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher_->knnMatch(prev_frame_.descriptors, cur.descriptors, knn_matches, 2);

    std::vector<cv::DMatch> good;
    for (auto& m : knn_matches) {
        if (m.size() >= 2 && m[0].distance < 0.75f * m[1].distance) good.push_back(m[0]);
    }

    res.initialized = true;

    if (good.size() < 8) {
        prev_frame_ = cur;
        trajectory_.push_back(pose_);
        return res;
    }

    // ── Essential matrix ───────────────────────────────────────────────────
    std::vector<cv::Point2f> pts1, pts2;
    for (auto& m : good) {
        pts1.push_back(prev_frame_.keypoints[m.queryIdx].pt);
        pts2.push_back(cur.keypoints[m.trainIdx].pt);
    }

    cv::Mat mask;
    cv::Mat E = cv::findEssentialMat(pts1, pts2, K_, cv::RANSAC, 0.999, 1.0, mask);
    if (E.empty()) {
        prev_frame_ = cur;
        trajectory_.push_back(pose_);
        return res;
    }

    cv::Mat R_rel, t_rel;
    int inliers = cv::recoverPose(E, pts1, pts2, K_, R_rel, t_rel, mask);
    res.num_inliers = inliers;

    // Keep only inlier matches for visualisation
    for (int i = int(good.size()) - 1; i >= 0; --i) {
        if (!mask.at<uchar>(i)) good.erase(good.begin() + i);
    }
    res.matches = good;

    if (inliers < 8) {
        prev_frame_ = cur;
        trajectory_.push_back(pose_);
        return res;
    }

    // ── Accumulate pose (scale unknown → unit translation) ─────────────────
    // World pose: p_world = R_prev * t_rel + t_prev
    //             R_world = R_prev * R_rel
    pose_.t = pose_.t + pose_.R * t_rel;
    pose_.R = pose_.R * R_rel;

    res.pose = pose_;
    trajectory_.push_back(pose_);
    prev_frame_ = cur;
    return res;
}

}  // namespace falconguide::vo
