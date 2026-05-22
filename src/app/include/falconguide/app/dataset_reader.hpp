#pragma once

#include "falconguide/app/config_json.hpp"
#include "falconguide/io/measurement_reader.hpp"

#include <memory>
#include <optional>
#include <string>

namespace falconguide::app {

// ── IDatasetReader ────────────────────────────────────────────────────────────
//
// Wraps IMeasurementReader and adds Rewind() for repeated playback.
// Future dataset types (ROS bag, mcap) implement this interface.
//
class IDatasetReader {
 public:
  virtual ~IDatasetReader() = default;

  virtual bool        Open()   = 0;
  virtual void        Close()  = 0;
  virtual bool        Rewind() = 0;   // seek back to beginning
  virtual io::ReadOutcome Next()  = 0;

  [[nodiscard]] virtual std::string Name()             const = 0;
  [[nodiscard]] virtual std::size_t MeasurementsRead() const = 0;
};

// ── Factory ───────────────────────────────────────────────────────────────────

std::unique_ptr<IDatasetReader> MakeDatasetReader(const DatasetConfig& cfg);

}  // namespace falconguide::app
