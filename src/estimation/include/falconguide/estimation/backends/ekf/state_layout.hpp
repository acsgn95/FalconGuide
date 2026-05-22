#pragma once

/**
 * @file state_layout.hpp
 * @brief Dynamic error-state segment registration for EKF backends.
 */

#include <Eigen/Core>

#include <cassert>
#include <unordered_map>

namespace falconguide::estimation::ekf {

// ── StateSegmentId
// ────────────────────────────────────────────────────────────
//
// Identifies a named block inside the dynamic error-state vector.
// The standard 15-state INS uses the first five; optional segments are added
// at registration time to support extended configurations (tightly coupled
// GNSS clock, barometer bias, etc.).
//
enum class StateSegmentId {
  Position,  ///< ENU position error, 3 states.
  Velocity,  ///< ENU velocity error, 3 states.
  Attitude,  ///< Attitude error rotation vector, 3 states.
  AccelBias, ///< Accelerometer bias error, 3 states.
  GyroBias,  ///< Gyroscope bias error, 3 states.
  GnssClock, ///< Receiver clock bias and drift, 2 states.
  BaroBias,  ///< Barometer altitude bias, 1 state.
};

/// @brief Offset and length of a registered error-state segment.
struct StateSegment {
  int offset{0}; ///< First index in the full error-state vector.
  int size{0};   ///< Number of scalar states in the segment.
};

// ── StateLayout
// ───────────────────────────────────────────────────────────────
//
// Maps segment IDs to contiguous index ranges in the error-state vector.
// Segments are assigned in registration order. After all segments are
// registered the layout is effectively immutable — the estimator reads it
// to build F, G, H matrices without knowing which segments are active.
//
class StateLayout {
public:
  /// @brief Registers a segment with a fixed size.
  void Register(StateSegmentId id, int size) {
    assert(segments_.find(id) == segments_.end() &&
           "segment already registered");
    assert(size > 0 && "segment size must be positive");
    segments_[id] = StateSegment{total_size_, size};
    total_size_ += size;
  }

  /// @brief Returns true when a segment is active in this layout.
  [[nodiscard]] bool Has(StateSegmentId id) const {
    return segments_.count(id) > 0;
  }

  /// @brief Returns metadata for a registered segment.
  [[nodiscard]] const StateSegment &Get(StateSegmentId id) const {
    return segments_.at(id);
  }

  /// @brief Returns the total scalar dimension.
  [[nodiscard]] int TotalSize() const { return total_size_; }

  /// @brief Returns a zero vector with layout dimension.
  [[nodiscard]] Eigen::VectorXd ZeroVector() const {
    return Eigen::VectorXd::Zero(total_size_);
  }
  /// @brief Returns a zero square matrix with layout dimension.
  [[nodiscard]] Eigen::MatrixXd ZeroMatrix() const {
    return Eigen::MatrixXd::Zero(total_size_, total_size_);
  }
  /// @brief Returns an identity square matrix with layout dimension.
  [[nodiscard]] Eigen::MatrixXd IdentityMatrix() const {
    return Eigen::MatrixXd::Identity(total_size_, total_size_);
  }

private:
  std::unordered_map<StateSegmentId, StateSegment> segments_;
  int total_size_{0};
};

// ── Factory helpers
// ───────────────────────────────────────────────────────────

// Standard 15-state INS layout (pos + vel + att + accel bias + gyro bias).
/// @brief Creates the standard 15-state INS error layout.
inline StateLayout MakeStandardLayout() {
  StateLayout layout;
  layout.Register(StateSegmentId::Position, 3);
  layout.Register(StateSegmentId::Velocity, 3);
  layout.Register(StateSegmentId::Attitude, 3);
  layout.Register(StateSegmentId::AccelBias, 3);
  layout.Register(StateSegmentId::GyroBias, 3);
  return layout;
}

// 17-state layout: standard INS + GNSS receiver clock bias and drift.
// Required for tightly coupled GNSS integration.
/// @brief Creates the standard layout plus GNSS receiver clock states.
inline StateLayout MakeTightlyCoupledLayout() {
  StateLayout layout = MakeStandardLayout();
  layout.Register(StateSegmentId::GnssClock, 2);
  return layout;
}

} // namespace falconguide::estimation::ekf
