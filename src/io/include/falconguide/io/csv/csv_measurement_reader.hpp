#pragma once

/**
 * @file csv_measurement_reader.hpp
 * @brief CSV dataset reader for replay and tests.
 */

#include "falconguide/io/measurement_reader.hpp"

#include <fstream>
#include <string>

namespace falconguide::io {

// ── CsvMeasurementReader
// ──────────────────────────────────────────────────────
//
// Reads sensor measurements from a plain-text CSV file for replay and testing.
//
// File format (lines starting with '#' are comments):
//
//   IMU,<timestamp_ns>,<ax>,<ay>,<az>,<wx>,<wy>,<wz>
//     ax/ay/az : specific force (m/s²)
//     wx/wy/wz : angular rate (rad/s)
//
//   GNSS,<timestamp_ns>,<lat_deg>,<lon_deg>,<alt_m>,<ve>,<vn>,<vu>,<pos_sigma_m>,<vel_sigma_mps>
//     ve/vn/vu : ENU velocity (m/s)
//     pos_sigma_m / vel_sigma_mps : isotropic 1-sigma noise
//
//   BARO,<timestamp_ns>,<pressure_pa>,<altitude_m>
//
//   MAG,<timestamp_ns>,<bx>,<by>,<bz>
//     bx/by/bz : magnetic field in sensor frame (Tesla)
//
// Measurements must be in ascending timestamp order.
// Unknown type tokens are silently skipped.
//
// Example:
//   CsvMeasurementReader reader("flight_log.csv");
//   reader.Open();
//   while (true) {
//     auto [result, meas, _] = reader.Next();
//     if (result != ReadResult::Ok) break;
//     pipeline.Push(*meas);
//   }
//
class CsvMeasurementReader : public IMeasurementReader {
   public:
    /// @brief Creates a CSV reader for a file path.
    explicit CsvMeasurementReader(std::string path, std::string name = "CsvMeasurementReader");

    /// @copydoc IMeasurementReader::Name
    [[nodiscard]] std::string Name() const override;
    /// @copydoc IMeasurementReader::Capabilities
    [[nodiscard]] ReaderCapabilities Capabilities() const override;

    /// @copydoc IMeasurementReader::Open
    bool Open() override;
    /// @copydoc IMeasurementReader::Close
    void Close() override;
    /// @copydoc IMeasurementReader::IsOpen
    [[nodiscard]] bool IsOpen() const override;

    /// @copydoc IMeasurementReader::Next
    ReadOutcome Next() override;

    /// @copydoc IMeasurementReader::StartTime
    [[nodiscard]] std::optional<core::Timestamp> StartTime() const override;
    /// @copydoc IMeasurementReader::EndTime
    [[nodiscard]] std::optional<core::Timestamp> EndTime() const override;

   private:
    static ReadOutcome ParseLine(const std::string &line);

    std::string path_;
    std::string name_;
    std::ifstream file_;

    std::optional<core::Timestamp> start_time_;
    std::optional<core::Timestamp> end_time_;
};

}  // namespace falconguide::io
