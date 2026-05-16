#pragma once

#include "falconguide/core/time.hpp"
#include "falconguide/estimation/estimator_interface.hpp"

#include <optional>
#include <string>

namespace falconguide::io {

// Describes what data a reader provides (bitmask-friendly flags).
enum class ReaderCapabilities : unsigned {
  None          = 0,
  Imu           = 1 << 0,
  Gnss          = 1 << 1,
  Camera        = 1 << 2,
  Magnetometer  = 1 << 3,
  Barometer     = 1 << 4,
  Odometry      = 1 << 5,
  RangeSensors  = 1 << 6,
  ExternalPose  = 1 << 7,
};

constexpr ReaderCapabilities operator|(ReaderCapabilities a, ReaderCapabilities b) noexcept {
  return static_cast<ReaderCapabilities>(static_cast<unsigned>(a) | static_cast<unsigned>(b));
}
constexpr bool HasCapability(ReaderCapabilities caps, ReaderCapabilities flag) noexcept {
  return (static_cast<unsigned>(caps) & static_cast<unsigned>(flag)) != 0;
}

// Reader state returned on each call to Next().
enum class ReadResult {
  Ok,         // measurement returned successfully
  EndOfData,  // no more data (EOF / stream closed)
  Timeout,    // live source: no data within deadline
  Error,      // unrecoverable parse or IO error
};

struct ReadOutcome {
  ReadResult result{ReadResult::EndOfData};
  std::optional<estimation::SensorMeasurement> measurement;
  std::string error_message;
};

// ── IMeasurementReader ────────────────────────────────────────────────────────
//
// Pull-based source of sensor measurements. Implementations may wrap a dataset
// file, a live serial/UDP stream, a ROS bag, etc.  The caller drives the loop:
//
//   while (true) {
//     auto [result, meas, _] = reader->Next();
//     if (result != ReadResult::Ok) break;
//     estimator->AddMeasurement(*meas);
//   }
//
class IMeasurementReader {
 public:
  virtual ~IMeasurementReader() = default;

  // Human-readable name for logging (e.g. "EuRoC MH01", "u-blox F9P").
  [[nodiscard]] virtual std::string Name() const = 0;

  [[nodiscard]] virtual ReaderCapabilities Capabilities() const = 0;

  // Open / connect the source. Returns false and sets error on failure.
  virtual bool Open() = 0;
  virtual void Close() = 0;
  [[nodiscard]] virtual bool IsOpen() const = 0;

  // Retrieve the next available measurement in timestamp order.
  // For live sources this may block up to an implementation-defined timeout.
  virtual ReadOutcome Next() = 0;

  // Seek to a specific time (dataset sources only). Live sources return false.
  virtual bool SeekTo(const core::Timestamp& /*timestamp*/) { return false; }

  // Earliest and latest timestamps known to the source (dataset only).
  [[nodiscard]] virtual std::optional<core::Timestamp> StartTime() const { return std::nullopt; }
  [[nodiscard]] virtual std::optional<core::Timestamp> EndTime()   const { return std::nullopt; }
};

}  // namespace falconguide::io
