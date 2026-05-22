#pragma once

#include "falconguide/io/measurement_writer.hpp"

#include <fstream>
#include <memory>
#include <ostream>
#include <string>

namespace falconguide::io {

// ── NmeaWriter ────────────────────────────────────────────────────────────────
//
// Writes NavigationState as NMEA 0183 sentences (FGGGA + FGRMC + FGVTG)
// to any std::ostream (file, stdout, TCP stream, etc.).
//
// Each WriteState() call emits three sentences:
//   FGGGA — position, altitude, fix quality
//   FGRMC — position, speed over ground, course over ground
//   FGVTG — course and speed
//
// Time strings are derived from system clock when GPS time is unavailable.
//
// Usage:
//   auto writer = NmeaWriter::ToFile("/dev/ttyUSB0");
//   writer->Open();
//   // ... in observer:
//   writer->WriteState(*nav_state);
//
class NmeaWriter : public IMeasurementWriter {
 public:
  // Write to an externally-owned stream (e.g. std::cout).
  // The caller must keep the stream alive for the lifetime of this writer.
  explicit NmeaWriter(std::ostream& stream, std::string name = "NmeaWriter");

  // Write to a file path.  Open() creates/truncates the file.
  static std::unique_ptr<NmeaWriter> ToFile(const std::string& path,
                                            std::string name = "NmeaWriter");

  [[nodiscard]] std::string Name() const override;

  bool Open()  override;
  void Close() override;
  [[nodiscard]] bool IsOpen() const override;

  WriteResult WriteState(const core::NavigationState& state) override;
  void Flush() override;

 private:
  static std::string FormatUtcTime();
  static std::string FormatUtcDate();

  std::string       name_;
  std::ostream*     stream_{nullptr};      // non-owning (stream constructor)
  std::string       file_path_;           // owning (ToFile constructor)
  std::unique_ptr<std::ofstream> owned_;  // owned file stream
  bool              open_{false};
};

}  // namespace falconguide::io
