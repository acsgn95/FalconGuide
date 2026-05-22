#pragma once

/**
 * @file dataset_reader.hpp
 * @brief Dataset reader abstraction and factory.
 */

#include "falconguide/app/config_json.hpp"
#include "falconguide/io/measurement_reader.hpp"

#include <memory>
#include <optional>
#include <string>

namespace falconguide::app {

// ── IDatasetReader
// ────────────────────────────────────────────────────────────
//
// Wraps IMeasurementReader and adds Rewind() for repeated playback.
// Future dataset types (ROS bag, mcap) implement this interface.
//
class IDatasetReader {
   public:
    /// @brief Virtual destructor for interface use.
    virtual ~IDatasetReader() = default;

    /// @brief Opens the dataset source.
    virtual bool Open() = 0;
    /// @brief Closes the dataset source.
    virtual void Close() = 0;
    /// @brief Rewinds the dataset to the beginning.
    virtual bool Rewind() = 0;  // seek back to beginning
    /// @brief Reads the next dataset measurement.
    virtual io::ReadOutcome Next() = 0;

    /// @brief Human-readable dataset reader name.
    [[nodiscard]] virtual std::string Name() const = 0;
    /// @brief Number of measurements emitted so far.
    [[nodiscard]] virtual std::size_t MeasurementsRead() const = 0;
};

// ── Factory
// ───────────────────────────────────────────────────────────────────

/// @brief Creates a dataset reader from dataset configuration.
std::unique_ptr<IDatasetReader> MakeDatasetReader(const DatasetConfig &cfg);

}  // namespace falconguide::app
