#pragma once

/**
 * @file measurement_writer.hpp
 * @brief Push-based writer abstraction for navigation outputs and measurements.
 */

#include "falconguide/core/navigation_state.hpp"
#include "falconguide/estimation/estimator_interface.hpp"

#include <string>

namespace falconguide::io {

enum class WriteResult {
    Ok,          ///< Write completed successfully.
    BufferFull,  ///< Back-pressure: caller should retry or drop.
    Error,       ///< Unrecoverable write error.
};

// ── IMeasurementWriter
// ────────────────────────────────────────────────────────
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
    /// @brief Virtual destructor for interface use.
    virtual ~IMeasurementWriter() = default;

    /// @brief Human-readable writer name.
    [[nodiscard]] virtual std::string Name() const = 0;

    /// @brief Opens the output sink.
    virtual bool Open() = 0;
    /// @brief Closes the output sink.
    virtual void Close() = 0;
    /// @brief Returns true when the sink is open.
    [[nodiscard]] virtual bool IsOpen() const = 0;

    /// @brief Writes a computed navigation solution.
    virtual WriteResult WriteState(const core::NavigationState &state) = 0;

    /// @brief Optionally writes a raw sensor measurement.
    virtual WriteResult WriteMeasurement(const estimation::SensorMeasurement & /*measurement*/) {
        return WriteResult::Ok;
    }

    /// @brief Flushes any internal buffers to the underlying sink.
    virtual void Flush() {}
};

}  // namespace falconguide::io
