#pragma once

#include "falconguide/core/frames.hpp"
#include "falconguide/core/time.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <map>
#include <vector>

namespace falconguide::estimation::gtsam_backend {

// ── Factor Graph Node Types ───────────────────────────────────────────────────

// Unique key for a pose / velocity / bias node in the factor graph.
// Encodes sensor type and sequence number — matches GTSAM's Symbol convention.
struct GraphKey {
  char symbol{'x'};       // 'x'=pose, 'v'=velocity, 'b'=bias, 'l'=landmark
  std::uint64_t index{0};

  bool operator<(const GraphKey& other) const {
    return symbol != other.symbol ? symbol < other.symbol : index < other.index;
  }
};

// Navigation state at one graph node.
struct NavNode {
  core::Timestamp timestamp;

  core::Vec3<core::EnuFrame>  position_enu_m;
  Eigen::Quaterniond          orientation_body_to_enu{Eigen::Quaterniond::Identity()};
  core::Vec3<core::EnuFrame>  velocity_enu_mps;
  core::Vec3<core::ImuFrame>  accel_bias_mps2;
  core::Vec3<core::ImuFrame>  gyro_bias_radps;

  GraphKey pose_key{'x', 0};
  GraphKey velocity_key{'v', 0};
  GraphKey bias_key{'b', 0};
};

// ── IMU Preintegration ────────────────────────────────────────────────────────
//
// GTSAM uses CombinedImuFactor which jointly preintegrates position, velocity,
// and rotation with a combined noise model (CombinedPreintegrationParams).
// This struct mirrors the inputs required to construct that factor.
//
struct GtsamImuPreintegration {
  core::Timestamp start_timestamp;
  core::Timestamp end_timestamp;

  // Accumulated delta in the IMU frame.
  Eigen::Vector3d   delta_p{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond delta_q{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d   delta_v{Eigen::Vector3d::Zero()};

  // Combined covariance [9×9: dp, dtheta, dv].
  Eigen::Matrix<double, 9, 9> covariance{Eigen::Matrix<double, 9, 9>::Zero()};

  GraphKey from_pose_key;
  GraphKey to_pose_key;
};

// ── Smoother Mode ─────────────────────────────────────────────────────────────

enum class SmootherMode {
  // iSAM2: incremental, Bayes tree-based. Best for real-time operation.
  // Relinearises only affected variables on each update — O(1) amortised.
  ISAM2,

  // Fixed-lag smoother: maintains only the last `lag_s` seconds of the graph.
  // Marginalises older states out similarly to Ceres sliding window.
  // Better bounded memory; slightly less accurate than full iSAM2.
  FixedLag,

  // Batch ISAM (offline). Relinearises the full graph on each update.
  // Only suitable for post-processing / dataset evaluation.
  Batch,
};

// ── iSAM2 Relinearisation Policy ─────────────────────────────────────────────

struct ISAM2Policy {
  // Relinearise a variable if its linearisation point has changed by more
  // than this threshold (pose: rad / translation: m, bias: matching units).
  double relinearise_threshold_pose{0.01};
  double relinearise_threshold_velocity{0.001};
  double relinearise_threshold_bias{0.0001};

  // Number of iterations of back-substitution per iSAM2 update.
  int relinearise_skip{10};

  // Enable CHOLESKY (faster) or QR (more numerically stable) factorisation.
  bool use_cholesky{true};
};

// ── Full Factor Graph State ───────────────────────────────────────────────────

struct FactorGraphState {
  SmootherMode mode{SmootherMode::ISAM2};
  ISAM2Policy  isam2_policy;
  double       fixed_lag_s{5.0};   // only used in FixedLag mode

  // All navigation nodes, keyed by pose GraphKey index (monotonically increasing).
  std::map<std::uint64_t, NavNode> nodes;

  // Preintegrated IMU factors, one per consecutive node pair.
  std::vector<GtsamImuPreintegration> imu_factors;

  // Running node index counter.
  std::uint64_t next_index{0};
};

}  // namespace falconguide::estimation::gtsam_backend
