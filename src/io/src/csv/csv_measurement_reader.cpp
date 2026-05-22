#include "falconguide/io/csv/csv_measurement_reader.hpp"

#include "falconguide/core/coordinates.hpp"
#include "falconguide/core/math.hpp"
#include "falconguide/core/sensors/environment.hpp"
#include "falconguide/core/sensors/gnss.hpp"
#include "falconguide/core/sensors/imu.hpp"
#include "falconguide/estimation/measurement_variant_helpers.hpp"

#include <charconv>
#include <chrono>
#include <sstream>

namespace falconguide::io {

namespace {

// Split a CSV line into tokens; handles empty fields.
std::vector<std::string> Split(const std::string& line, char delim = ',') {
  std::vector<std::string> tokens;
  std::istringstream ss(line);
  std::string token;
  while (std::getline(ss, token, delim)) {
    tokens.push_back(token);
  }
  return tokens;
}

double ToDouble(const std::string& s) { return std::stod(s); }

std::int64_t ToInt64(const std::string& s) { return std::stoll(s); }

core::Timestamp MakeTimestamp(std::int64_t ns) {
  core::Timestamp t;
  t.has_steady = true;
  t.steady     = core::MonotonicTime(std::chrono::nanoseconds(ns));
  return t;
}

}  // namespace

// ── Constructor ───────────────────────────────────────────────────────────────

CsvMeasurementReader::CsvMeasurementReader(std::string path, std::string name)
    : path_(std::move(path)), name_(std::move(name)) {}

// ── IMeasurementReader ────────────────────────────────────────────────────────

std::string CsvMeasurementReader::Name() const { return name_; }

ReaderCapabilities CsvMeasurementReader::Capabilities() const {
  return ReaderCapabilities::Imu
       | ReaderCapabilities::Gnss
       | ReaderCapabilities::Barometer
       | ReaderCapabilities::Magnetometer;
}

bool CsvMeasurementReader::Open() {
  file_.open(path_);
  return file_.is_open();
}

void CsvMeasurementReader::Close() { file_.close(); }

bool CsvMeasurementReader::IsOpen() const { return file_.is_open(); }

std::optional<core::Timestamp> CsvMeasurementReader::StartTime() const {
  return start_time_;
}

std::optional<core::Timestamp> CsvMeasurementReader::EndTime() const {
  return end_time_;
}

// ── Next ──────────────────────────────────────────────────────────────────────

ReadOutcome CsvMeasurementReader::Next() {
  if (!file_.is_open()) return {ReadResult::Error, std::nullopt, "file not open"};

  std::string line;
  while (std::getline(file_, line)) {
    // Skip empty lines and comments
    if (line.empty() || line.front() == '#') continue;

    ReadOutcome outcome = ParseLine(line);

    if (outcome.result == ReadResult::Ok && outcome.measurement) {
      // Track time bounds
      const auto ts = falconguide::estimation::GetTimestamp(*outcome.measurement);
      if (!start_time_) start_time_ = ts;
      end_time_ = ts;
      return outcome;
    }
    // Unknown / malformed lines are silently skipped
  }

  return {ReadResult::EndOfData};
}

// ── ParseLine ─────────────────────────────────────────────────────────────────

ReadOutcome CsvMeasurementReader::ParseLine(const std::string& line) {
  const auto tokens = Split(line);
  if (tokens.empty()) return {ReadResult::EndOfData};

  const std::string& type = tokens[0];

  try {
    // ── IMU ───────────────────────────────────────────────────────────────────
    // IMU,<ns>,<ax>,<ay>,<az>,<wx>,<wy>,<wz>
    if (type == "IMU" && tokens.size() >= 8) {
      core::ImuMeasurement imu;
      imu.timestamp = MakeTimestamp(ToInt64(tokens[1]));
      imu.specific_force_mps2 = core::Vec3<core::ImuFrame>(
          ToDouble(tokens[2]), ToDouble(tokens[3]), ToDouble(tokens[4]));
      imu.angular_rate_radps = core::Vec3<core::ImuFrame>(
          ToDouble(tokens[5]), ToDouble(tokens[6]), ToDouble(tokens[7]));
      imu.validity = core::MeasurementValidity::Valid;
      return {ReadResult::Ok, imu};
    }

    // ── GNSS ──────────────────────────────────────────────────────────────────
    // GNSS,<ns>,<lat_deg>,<lon_deg>,<alt_m>,<ve>,<vn>,<vu>,<pos_sigma_m>,<vel_sigma_mps>
    if (type == "GNSS" && tokens.size() >= 10) {
      const double lat = core::DegToRad(ToDouble(tokens[2]));
      const double lon = core::DegToRad(ToDouble(tokens[3]));
      const double alt = ToDouble(tokens[4]);

      const double ve = ToDouble(tokens[5]);
      const double vn = ToDouble(tokens[6]);
      const double vu = ToDouble(tokens[7]);

      const double pos_sigma = ToDouble(tokens[8]);
      const double vel_sigma = ToDouble(tokens[9]);

      core::GnssSolution gnss;
      gnss.timestamp        = MakeTimestamp(ToInt64(tokens[1]));
      gnss.position_ecef_m  = core::LlaToEcef(core::Lla{lat, lon, alt});
      gnss.fix_type         = core::GnssFixType::Single;
      gnss.validity         = core::MeasurementValidity::Valid;

      // ENU velocity → ECEF via local tangent plane rotation
      const core::LocalTangentPlane ltp(core::Lla{lat, lon, alt});
      const Eigen::Matrix3d R_enu_to_ecef = ltp.ecef_to_enu_rotation().transpose();
      gnss.velocity_ecef_mps = core::Vec3<core::EcefFrame>(
          R_enu_to_ecef * Eigen::Vector3d(ve, vn, vu));

      gnss.position_covariance_ecef_m2     = Eigen::Matrix3d::Identity() * (pos_sigma * pos_sigma);
      gnss.velocity_covariance_ecef_m2ps2  = Eigen::Matrix3d::Identity() * (vel_sigma * vel_sigma);
      return {ReadResult::Ok, gnss};
    }

    // ── Barometer ─────────────────────────────────────────────────────────────
    // BARO,<ns>,<pressure_pa>,<altitude_m>
    if (type == "BARO" && tokens.size() >= 4) {
      core::BarometerMeasurement baro;
      baro.timestamp   = MakeTimestamp(ToInt64(tokens[1]));
      baro.pressure_pa = ToDouble(tokens[2]);
      baro.altitude_m  = ToDouble(tokens[3]);
      baro.validity    = core::MeasurementValidity::Valid;
      return {ReadResult::Ok, baro};
    }

    // ── Magnetometer ──────────────────────────────────────────────────────────
    // MAG,<ns>,<bx>,<by>,<bz>
    if (type == "MAG" && tokens.size() >= 5) {
      core::MagnetometerMeasurement mag;
      mag.timestamp = MakeTimestamp(ToInt64(tokens[1]));
      mag.magnetic_field_tesla = core::Vec3<core::MagnetometerFrame>(
          ToDouble(tokens[2]), ToDouble(tokens[3]), ToDouble(tokens[4]));
      mag.validity = core::MeasurementValidity::Valid;
      return {ReadResult::Ok, mag};
    }

  } catch (const std::exception& ex) {
    return {ReadResult::Error, std::nullopt, std::string("parse error: ") + ex.what()};
  }

  // Unknown type — skip silently
  return {ReadResult::EndOfData};
}

}  // namespace falconguide::io
