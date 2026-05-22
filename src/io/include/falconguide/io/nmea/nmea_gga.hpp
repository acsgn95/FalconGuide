#pragma once

#include "falconguide/core/coordinates.hpp"
#include "falconguide/core/math.hpp"
#include "falconguide/core/navigation_state.hpp"
#include "falconguide/core/sensors/gnss.hpp"
#include "falconguide/io/nmea/nmea_sentence.hpp"

#include <cmath>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace falconguide::io::nmea {

[[nodiscard]] inline std::optional<double> ParseNmeaLatitudeLongitude(std::string_view value, std::string_view hemisphere,
                                                                      bool is_latitude) {
  if (value.empty() || hemisphere.empty()) {
    return std::nullopt;
  }

  const int degree_digits = is_latitude ? 2 : 3;
  if (value.size() <= static_cast<std::size_t>(degree_digits)) {
    return std::nullopt;
  }

  const double degrees = std::stod(std::string(value.substr(0, degree_digits)));
  const double minutes = std::stod(std::string(value.substr(degree_digits)));
  double decimal_degrees = degrees + minutes / 60.0;

  if (hemisphere == "S" || hemisphere == "W") {
    decimal_degrees = -decimal_degrees;
  } else if (!(hemisphere == "N" || hemisphere == "E")) {
    return std::nullopt;
  }

  return core::DegToRad(decimal_degrees);
}

[[nodiscard]] inline std::string FormatNmeaLatitudeLongitude(double radians, bool is_latitude) {
  const double absolute_degrees = std::abs(core::RadToDeg(radians));
  const int degrees = static_cast<int>(std::floor(absolute_degrees));
  const double minutes = (absolute_degrees - static_cast<double>(degrees)) * 60.0;
  const int degree_width = is_latitude ? 2 : 3;

  std::ostringstream stream;
  stream << std::setw(degree_width) << std::setfill('0') << degrees
         << std::fixed << std::setprecision(7) << std::setw(10) << std::setfill('0') << minutes;
  return stream.str();
}

[[nodiscard]] inline core::GnssFixType ParseGgaFixType(const std::string& field) {
  if (field == "1") {
    return core::GnssFixType::Single;
  }
  if (field == "2") {
    return core::GnssFixType::Differential;
  }
  if (field == "4") {
    return core::GnssFixType::RtkFixed;
  }
  if (field == "5") {
    return core::GnssFixType::RtkFloat;
  }
  return core::GnssFixType::NoFix;
}

[[nodiscard]] inline std::string FormatGgaFixType(core::NavigationStatus status) {
  if (status == core::NavigationStatus::Nominal || status == core::NavigationStatus::Degraded ||
      status == core::NavigationStatus::DeadReckoning) {
    return "1";
  }
  return "0";
}

[[nodiscard]] inline std::optional<core::GnssSolution> ParseGga(const NmeaSentence& sentence) {
  if (sentence.formatter != "GGA" || sentence.fields.size() < 14) {
    return std::nullopt;
  }

  const std::optional<double> latitude =
      ParseNmeaLatitudeLongitude(sentence.fields[1], sentence.fields[2], true);
  const std::optional<double> longitude =
      ParseNmeaLatitudeLongitude(sentence.fields[3], sentence.fields[4], false);
  if (!latitude || !longitude) {
    return std::nullopt;
  }

  const double altitude_m = sentence.fields[8].empty() ? 0.0 : std::stod(sentence.fields[8]);
  core::GnssSolution solution;
  solution.position_ecef_m = core::LlaToEcef(core::Lla{*latitude, *longitude, altitude_m});
  solution.fix_type = ParseGgaFixType(sentence.fields[5]);
  solution.horizontal_dop = sentence.fields[7].empty() ? std::optional<double>{} : std::stod(sentence.fields[7]);
  solution.validity =
      solution.fix_type == core::GnssFixType::NoFix ? core::MeasurementValidity::Unknown : core::MeasurementValidity::Valid;
  return solution;
}

[[nodiscard]] inline std::optional<core::GnssSolution> ParseGgaLine(std::string_view line) {
  const std::optional<NmeaSentence> sentence = ParseSentence(line);
  if (!sentence) {
    return std::nullopt;
  }
  return ParseGga(*sentence);
}

[[nodiscard]] inline std::string WriteGga(const core::NavigationState& state, std::string_view utc_time = "") {
  const core::Lla lla = core::EcefToLla(state.position_ecef_m);
  const std::string latitude = FormatNmeaLatitudeLongitude(lla.latitude_rad, true);
  const std::string longitude = FormatNmeaLatitudeLongitude(lla.longitude_rad, false);

  std::ostringstream altitude;
  altitude << std::fixed << std::setprecision(2) << lla.altitude_m;

  std::ostringstream hdop;
  if (state.quality.horizontal_accuracy_m) {
    hdop << std::fixed << std::setprecision(1) << *state.quality.horizontal_accuracy_m;
  }

  const std::vector<std::string> fields{
      std::string(utc_time),
      latitude,
      lla.latitude_rad >= 0.0 ? "N" : "S",
      longitude,
      lla.longitude_rad >= 0.0 ? "E" : "W",
      FormatGgaFixType(state.status),
      "00",
      hdop.str(),
      altitude.str(),
      "M",
      "0.00",
      "M",
      "",
      ""};

  return BuildSentence("FG", "GGA", fields);
}

}  // namespace falconguide::io::nmea
