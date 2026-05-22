#pragma once

#include <Eigen/Core>

#include <cassert>
#include <unordered_map>

namespace falconguide::estimation::ekf {

// ── StateSegmentId ────────────────────────────────────────────────────────────
//
// Identifies a named block inside the dynamic error-state vector.
// The standard 15-state INS uses the first five; optional segments are added
// at registration time to support extended configurations (tightly coupled
// GNSS clock, barometer bias, etc.).
//
enum class StateSegmentId {
  Position,   // 3: δp  — ENU position error (m)
  Velocity,   // 3: δv  — ENU velocity error (m/s)
  Attitude,   // 3: δθ  — attitude error as rotation vector, body frame (rad)
  AccelBias,  // 3: δba — accelerometer bias error, body frame (m/s²)
  GyroBias,   // 3: δbg — gyroscope bias error, body frame (rad/s)
  GnssClock,  // 2: [δt_b, δt_d] — receiver clock bias (m) + drift (m/s)
  BaroBias,   // 1: δb_baro — barometer altitude bias (m)
};

struct StateSegment {
  int offset{0};
  int size{0};
};

// ── StateLayout ───────────────────────────────────────────────────────────────
//
// Maps segment IDs to contiguous index ranges in the error-state vector.
// Segments are assigned in registration order. After all segments are
// registered the layout is effectively immutable — the estimator reads it
// to build F, G, H matrices without knowing which segments are active.
//
class StateLayout {
 public:
  void Register(StateSegmentId id, int size) {
    assert(segments_.find(id) == segments_.end() && "segment already registered");
    assert(size > 0 && "segment size must be positive");
    segments_[id] = StateSegment{total_size_, size};
    total_size_ += size;
  }

  [[nodiscard]] bool Has(StateSegmentId id) const {
    return segments_.count(id) > 0;
  }

  [[nodiscard]] const StateSegment& Get(StateSegmentId id) const {
    return segments_.at(id);
  }

  [[nodiscard]] int TotalSize() const { return total_size_; }

  [[nodiscard]] Eigen::VectorXd ZeroVector()    const { return Eigen::VectorXd::Zero(total_size_); }
  [[nodiscard]] Eigen::MatrixXd ZeroMatrix()    const { return Eigen::MatrixXd::Zero(total_size_, total_size_); }
  [[nodiscard]] Eigen::MatrixXd IdentityMatrix() const { return Eigen::MatrixXd::Identity(total_size_, total_size_); }

 private:
  std::unordered_map<StateSegmentId, StateSegment> segments_;
  int total_size_{0};
};

// ── Factory helpers ───────────────────────────────────────────────────────────

// Standard 15-state INS layout (pos + vel + att + accel bias + gyro bias).
inline StateLayout MakeStandardLayout() {
  StateLayout layout;
  layout.Register(StateSegmentId::Position,  3);
  layout.Register(StateSegmentId::Velocity,  3);
  layout.Register(StateSegmentId::Attitude,  3);
  layout.Register(StateSegmentId::AccelBias, 3);
  layout.Register(StateSegmentId::GyroBias,  3);
  return layout;
}

// 17-state layout: standard INS + GNSS receiver clock bias and drift.
// Required for tightly coupled GNSS integration.
inline StateLayout MakeTightlyCoupledLayout() {
  StateLayout layout = MakeStandardLayout();
  layout.Register(StateSegmentId::GnssClock, 2);
  return layout;
}

}  // namespace falconguide::estimation::ekf
