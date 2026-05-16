#pragma once

#include "falconguide/core/navigation_state.hpp"
#include "falconguide/estimation/estimator_interface.hpp"

#include <string>

namespace falconguide::io {

enum class WriteResult {
  Ok,
  BufferFull,  // back-pressure: caller should retry or drop
  Error,
};

// ── IMeasurementWriter ────────────────────────────────────────────────────────
//
// Push-based sink for navigation outputs or raw measurements.
// Implementations may write to a file, a serial port, ROS topic, network, etc.
//
// Two orthogonal write paths:
//   WriteState  — NavigationState output (position / velocity / attitude)
//   WriteMeasurement — echo raw sensor data (logging, recording)
//
class IMeasurementWriter {
 public:
  virtual ~IMeasurementWriter() = default;

  [[nodiscard]] virtual std::string Name() const = 0;

  virtual bool Open() = 0;
  virtual void Close() = 0;
  [[nodiscard]] virtual bool IsOpen() const = 0;

  // Write a computed navigation solution.
  virtual WriteResult WriteState(const core::NavigationState& state) = 0;

  // Optionally write a raw sensor measurement (for data recording).
  // Default: no-op (writers that only output navigation may skip this).
  virtual WriteResult WriteMeasurement(const estimation::SensorMeasurement& /*measurement*/) {
    return WriteResult::Ok;
  }

  // Flush any internal buffers to the underlying sink.
  virtual void Flush() {}
};

}  // namespace falconguide::io
