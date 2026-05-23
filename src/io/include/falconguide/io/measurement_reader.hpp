#pragma once

/**
 * @file measurement_reader.hpp
 * @brief Pull-based sensor measurement reader abstraction.
 */

#include "falconguide/core/time.hpp"
#include "falconguide/estimation/estimator_interface.hpp"

#include <optional>
#include <string>

namespace falconguide::io {

/// @brief Bitmask-friendly flags describing what data a reader provides.
enum class ReaderCapabilities : unsigned {
    None = 0,               ///< No known capabilities.
    Imu = 1 << 0,           ///< IMU measurements.
    Gnss = 1 << 1,          ///< GNSS measurements.
    Camera = 1 << 2,        ///< Camera metadata.
    Magnetometer = 1 << 3,  ///< Magnetometer measurements.
    Barometer = 1 << 4,     ///< Barometer measurements.
    Odometry = 1 << 5,      ///< Odometry measurements.
    RangeSensors = 1 << 6,  ///< Range, altimeter, DVL, or sounder measurements.
    ExternalPose = 1 << 7,  ///< External pose or odometry measurements.
};

/// @brief Combines reader capability flags.
constexpr ReaderCapabilities operator|(ReaderCapabilities a, ReaderCapabilities b) noexcept {
    return static_cast<ReaderCapabilities>(static_cast<unsigned>(a) | static_cast<unsigned>(b));
}
/// @brief Tests whether a capability flag is present.
constexpr bool HasCapability(ReaderCapabilities caps, ReaderCapabilities flag) noexcept {
    return (static_cast<unsigned>(caps) & static_cast<unsigned>(flag)) != 0;
}

/// @brief Reader state returned on each call to Next().
enum class ReadResult {
    Ok,         ///< Measurement returned successfully.
    EndOfData,  ///< No more data, EOF, or stream closed.
    Timeout,    ///< Live source produced no data within deadline.
    Error,      ///< Unrecoverable parse or IO error.
};

/// @brief Result payload returned by IMeasurementReader::Next().
struct ReadOutcome {
    ReadResult result{ReadResult::EndOfData};                  ///< Read outcome.
    std::optional<estimation::SensorMeasurement> measurement;  ///< Measurement when result is Ok.
    std::optional<std::string> camera_frame_path;              ///< Camera frame path, when a new frame is available.
    std::string error_message;                                 ///< Error detail when result is Error.
};

// ── IMeasurementReader
// ────────────────────────────────────────────────────────
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
    /// @brief Virtual destructor for interface use.
    virtual ~IMeasurementReader() = default;

    /// @brief Human-readable name for logging.
    [[nodiscard]] virtual std::string Name() const = 0;

    /// @brief Returns source capabilities.
    [[nodiscard]] virtual ReaderCapabilities Capabilities() const = 0;

    /// @brief Opens or connects the source.
    /// @return False on failure.
    virtual bool Open() = 0;
    /// @brief Closes the source.
    virtual void Close() = 0;
    /// @brief Returns true when the source is open.
    [[nodiscard]] virtual bool IsOpen() const = 0;

    /// @brief Retrieves the next available measurement in timestamp order.
    virtual ReadOutcome Next() = 0;

    /// @brief Seeks to a specific time for dataset sources.
    /// @return False for live or non-seekable sources.
    virtual bool SeekTo(const core::Timestamp& /*timestamp*/) { return false; }

    /// @brief Earliest timestamp known to the source, when available.
    [[nodiscard]] virtual std::optional<core::Timestamp> StartTime() const { return std::nullopt; }
    /// @brief Latest timestamp known to the source, when available.
    [[nodiscard]] virtual std::optional<core::Timestamp> EndTime() const { return std::nullopt; }
};

}  // namespace falconguide::io
