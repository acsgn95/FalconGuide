#pragma once

#include "falconguide/core/math.hpp"
#include "falconguide/core/navigation_state.hpp"
#include "falconguide/io/nmea/nmea_sentence.hpp"

#include <cmath>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace falconguide::io::nmea {

// VTG — Course and Speed over Ground.
// Fields: COG_true, T, COG_mag, M, speed_knots, N, speed_kph, K, [mode]

struct VtgData {
  double course_true_deg{0.0};   // course over ground, true north
  double speed_knots{0.0};
  double speed_kph{0.0};
};

[[nodiscard]] inline std::optional<VtgData> ParseVtg(const NmeaSentence& sentence) {
  if (sentence.formatter != "VTG" || sentence.fields.size() < 7) {
    return std::nullopt;
  }

  // Mode field (index 8 in NMEA 2.3+): 'N' means no fix / invalid.
  if (sentence.fields.size() >= 9 && sentence.fields[8] == "N") {
    return std::nullopt;
  }

  VtgData data;
  if (!sentence.fields[0].empty()) {
    data.course_true_deg = std::stod(sentence.fields[0]);
  }
  if (!sentence.fields[4].empty()) {
    data.speed_knots = std::stod(sentence.fields[4]);
  }
  if (!sentence.fields[6].empty()) {
    data.speed_kph = std::stod(sentence.fields[6]);
  }
  return data;
}

[[nodiscard]] inline std::optional<VtgData> ParseVtgLine(std::string_view line) {
  const std::optional<NmeaSentence> sentence = ParseSentence(line);
  if (!sentence) {
    return std::nullopt;
  }
  return ParseVtg(*sentence);
}

[[nodiscard]] inline std::string WriteVtg(const core::NavigationState& state) {
  // Course over ground from ENU velocity (atan2 east, north) -> clockwise from true north.
  const double vn = state.velocity_enu_mps.y();  // ENU: y = north
  const double ve = state.velocity_enu_mps.x();  // ENU: x = east

  double course_deg = core::RadToDeg(std::atan2(ve, vn));
  if (course_deg < 0.0) {
    course_deg += 360.0;
  }

  const double speed_mps = std::sqrt(ve * ve + vn * vn);
  constexpr double kMpsToKnots = 1.0 / 0.514444;
  constexpr double kMpsToKph   = 3.6;
  const double speed_knots = speed_mps * kMpsToKnots;
  const double speed_kph   = speed_mps * kMpsToKph;

  std::ostringstream cog;
  cog << std::fixed << std::setprecision(2) << course_deg;

  std::ostringstream spd_kt;
  spd_kt << std::fixed << std::setprecision(2) << speed_knots;

  std::ostringstream spd_kph;
  spd_kph << std::fixed << std::setprecision(2) << speed_kph;

  const bool valid = state.status == core::NavigationStatus::Nominal ||
                     state.status == core::NavigationStatus::Degraded ||
                     state.status == core::NavigationStatus::DeadReckoning;
  const std::string mode = valid ? "A" : "N";

  const std::vector<std::string> fields{
      cog.str(),   // course true
      "T",
      "",          // course magnetic — not computed
      "M",
      spd_kt.str(),
      "N",
      spd_kph.str(),
      "K",
      mode,
  };

  return BuildSentence("FG", "VTG", fields);
}

}  // namespace falconguide::io::nmea
