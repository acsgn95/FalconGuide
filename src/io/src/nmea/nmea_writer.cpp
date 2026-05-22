#include "falconguide/io/nmea/nmea_writer.hpp"
#include "falconguide/io/nmea/nmea_gga.hpp"
#include "falconguide/io/nmea/nmea_rmc.hpp"
#include "falconguide/io/nmea/nmea_vtg.hpp"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace falconguide::io {

// ── Stream constructor ────────────────────────────────────────────────────────

NmeaWriter::NmeaWriter(std::ostream& stream, std::string name) : name_(std::move(name)), stream_(&stream) {}

// ── File factory ──────────────────────────────────────────────────────────────

std::unique_ptr<NmeaWriter> NmeaWriter::ToFile(const std::string& path, std::string name) {
    auto w = std::make_unique<NmeaWriter>(std::cout, std::move(name));
    w->file_path_ = path;
    w->stream_ = nullptr;
    return w;
}

// ── Lifecycle ─────────────────────────────────────────────────────────────────

std::string NmeaWriter::Name() const { return name_; }

bool NmeaWriter::Open() {
    if (open_) return true;

    if (!file_path_.empty()) {
        owned_ = std::make_unique<std::ofstream>(file_path_, std::ios::out | std::ios::trunc);
        if (!owned_->is_open()) return false;
        stream_ = owned_.get();
    }

    open_ = (stream_ != nullptr);
    return open_;
}

void NmeaWriter::Close() {
    if (!open_) return;
    Flush();
    if (owned_) {
        owned_->close();
        owned_.reset();
        stream_ = nullptr;
    }
    open_ = false;
}

bool NmeaWriter::IsOpen() const { return open_; }

void NmeaWriter::Flush() {
    if (stream_) stream_->flush();
}

// ── WriteState ────────────────────────────────────────────────────────────────

WriteResult NmeaWriter::WriteState(const core::NavigationState& state) {
    if (!open_ || !stream_) return WriteResult::Error;

    const std::string utc_time = FormatUtcTime();
    const std::string utc_date = FormatUtcDate();

    *stream_ << nmea::WriteGga(state, utc_time) << "\r\n"
             << nmea::WriteRmc(state, utc_time, utc_date) << "\r\n"
             << nmea::WriteVtg(state) << "\r\n";

    return stream_->good() ? WriteResult::Ok : WriteResult::Error;
}

// ── Time helpers ──────────────────────────────────────────────────────────────

std::string NmeaWriter::FormatUtcTime() {
    const auto now = std::chrono::system_clock::now();
    const auto time_val = std::chrono::system_clock::to_time_t(now);
    const std::tm* t = std::gmtime(&time_val);

    std::ostringstream oss;
    oss << std::setfill('0') << std::setw(2) << t->tm_hour << std::setw(2) << t->tm_min << std::setw(2) << t->tm_sec
        << ".00";
    return oss.str();
}

std::string NmeaWriter::FormatUtcDate() {
    const auto now = std::chrono::system_clock::now();
    const auto time_val = std::chrono::system_clock::to_time_t(now);
    const std::tm* t = std::gmtime(&time_val);

    std::ostringstream oss;
    oss << std::setfill('0') << std::setw(2) << t->tm_mday << std::setw(2) << (t->tm_mon + 1) << std::setw(2)
        << (t->tm_year % 100);
    return oss.str();
}

}  // namespace falconguide::io
