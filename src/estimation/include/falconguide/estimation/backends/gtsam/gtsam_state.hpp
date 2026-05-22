#pragma once

/**
 * @file gtsam_state.hpp
 * @brief Factor-graph state containers used by the GTSAM backend.
 */

#include "falconguide/core/frames.hpp"
#include "falconguide/core/time.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <map>
#include <vector>

namespace falconguide::estimation::gtsam_backend {

// ── Factor Graph Node Types
// ───────────────────────────────────────────────────

// Unique key for a pose / velocity / bias node in the factor graph.
// Encodes sensor type and sequence number — matches GTSAM's Symbol convention.
struct GraphKey {
    char symbol{'x'};        ///< Symbol family, e.g. x=pose, v=velocity, b=bias.
    std::uint64_t index{0};  ///< Monotonic graph node index.

    /// @brief Provides strict weak ordering for map keys.
    bool operator<(const GraphKey &other) const {
        return symbol != other.symbol ? symbol < other.symbol : index < other.index;
    }
};

// Navigation state at one graph node.
struct NavNode {
    core::Timestamp timestamp;  ///< Node timestamp.

    core::Vec3<core::EnuFrame> position_enu_m;                                   ///< ENU position.
    Eigen::Quaterniond orientation_body_to_enu{Eigen::Quaterniond::Identity()};  ///< Body-to-ENU attitude.
    core::Vec3<core::EnuFrame> velocity_enu_mps;                                 ///< ENU velocity.
    core::Vec3<core::ImuFrame> accel_bias_mps2;                                  ///< Accelerometer bias.
    core::Vec3<core::ImuFrame> gyro_bias_radps;                                  ///< Gyroscope bias.

    GraphKey pose_key{'x', 0};      ///< GTSAM pose symbol.
    GraphKey velocity_key{'v', 0};  ///< GTSAM velocity symbol.
    GraphKey bias_key{'b', 0};      ///< GTSAM bias symbol.
};

// ── IMU Preintegration
// ────────────────────────────────────────────────────────
//
// GTSAM uses CombinedImuFactor which jointly preintegrates position, velocity,
// and rotation with a combined noise model (CombinedPreintegrationParams).
// This struct mirrors the inputs required to construct that factor.
//
struct GtsamImuPreintegration {
    core::Timestamp start_timestamp;  ///< Integration start timestamp.
    core::Timestamp end_timestamp;    ///< Integration end timestamp.

    // Accumulated delta in the IMU frame.
    Eigen::Vector3d delta_p{Eigen::Vector3d::Zero()};            ///< Preintegrated position delta.
    Eigen::Quaterniond delta_q{Eigen::Quaterniond::Identity()};  ///< Preintegrated rotation delta.
    Eigen::Vector3d delta_v{Eigen::Vector3d::Zero()};            ///< Preintegrated velocity delta.

    // Combined covariance [9×9: dp, dtheta, dv].
    Eigen::Matrix<double, 9, 9> covariance{Eigen::Matrix<double, 9, 9>::Zero()};  ///< Combined IMU covariance.

    GraphKey from_pose_key;  ///< Start pose key.
    GraphKey to_pose_key;    ///< End pose key.
};

// ── Smoother Mode
// ─────────────────────────────────────────────────────────────

enum class SmootherMode {
    // iSAM2: incremental, Bayes tree-based. Best for real-time operation.
    // Relinearises only affected variables on each update — O(1) amortised.
    ISAM2,  ///< Incremental Bayes-tree based iSAM2 smoother.

    // Fixed-lag smoother: maintains only the last `lag_s` seconds of the graph.
    // Marginalises older states out similarly to Ceres sliding window.
    // Better bounded memory; slightly less accurate than full iSAM2.
    FixedLag,  ///< Fixed-lag smoother with bounded graph horizon.

    // Batch ISAM (offline). Relinearises the full graph on each update.
    // Only suitable for post-processing / dataset evaluation.
    Batch,  ///< Batch ISAM mode for offline processing.
};

// ── iSAM2 Relinearisation Policy ─────────────────────────────────────────────

struct ISAM2Policy {
    // Relinearise a variable if its linearisation point has changed by more
    // than this threshold (pose: rad / translation: m, bias: matching units).
    double relinearise_threshold_pose{0.01};       ///< Pose relinearization threshold.
    double relinearise_threshold_velocity{0.001};  ///< Velocity relinearization threshold.
    double relinearise_threshold_bias{0.0001};     ///< Bias relinearization threshold.

    // Number of iterations of back-substitution per iSAM2 update.
    int relinearise_skip{10};  ///< Number of updates between relinearization checks.

    // Enable CHOLESKY (faster) or QR (more numerically stable) factorisation.
    bool use_cholesky{true};  ///< True for Cholesky factorization, false for QR.
};

// ── Full Factor Graph State
// ───────────────────────────────────────────────────

struct FactorGraphState {
    SmootherMode mode{SmootherMode::ISAM2};  ///< Active smoother mode.
    ISAM2Policy isam2_policy;                ///< iSAM2 relinearization policy.
    double fixed_lag_s{5.0};                 ///< Fixed-lag horizon in seconds.

    // All navigation nodes, keyed by pose GraphKey index (monotonically
    // increasing).
    std::map<std::uint64_t, NavNode> nodes;  ///< Navigation nodes keyed by index.

    // Preintegrated IMU factors, one per consecutive node pair.
    std::vector<GtsamImuPreintegration> imu_factors;  ///< IMU factors between consecutive nodes.

    // Running node index counter.
    std::uint64_t next_index{0};  ///< Next graph node index to allocate.
};

}  // namespace falconguide::estimation::gtsam_backend
