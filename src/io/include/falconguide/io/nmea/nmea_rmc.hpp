#pragma once

/**
 * @file nmea_rmc.hpp
 * @brief NMEA RMC parsing and FalconGuide RMC sentence generation.
 */

#include "falconguide/core/coordinates.hpp"
#include "falconguide/core/math.hpp"
#include "falconguide/core/navigation_state.hpp"
#include "falconguide/core/sensors/gnss.hpp"
#include "falconguide/io/nmea/nmea_gga.hpp"
#include "falconguide/io/nmea/nmea_sentence.hpp"

#include <cmath>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>

namespace falconguide::io::nmea {

/// @brief Converts NavigationStatus to an RMC active/void status character.
[[nodiscard]] inline char RmcStatusChar(core::NavigationStatus status) noexcept {
    switch (status) {
        case core::NavigationStatus::Nominal:
        case core::NavigationStatus::Degraded:
        case core::NavigationStatus::DeadReckoning:
            return 'A';
        default:
            return 'V';
    }
}

/// @brief Parses a validated RMC sentence into a GNSS solution.
[[nodiscard]] inline std::optional<core::GnssSolution> ParseRmc(const NmeaSentence &sentence) {
    // Fields: hhmmss.ss, status, lat, N/S, lon, E/W, speed_knots, course, ddmmyy,
    // mag_var, E/W, [mode]
    if (sentence.formatter != "RMC" || sentence.fields.size() < 10) {
        return std::nullopt;
    }

    if (sentence.fields[1] != "A") {
        return std::nullopt;  // void — no fix
    }

    const std::optional<double> latitude = ParseNmeaLatitudeLongitude(sentence.fields[2], sentence.fields[3], true);
    const std::optional<double> longitude = ParseNmeaLatitudeLongitude(sentence.fields[4], sentence.fields[5], false);
    if (!latitude || !longitude) {
        return std::nullopt;
    }

    core::GnssSolution solution;
    // RMC carries no altitude — use 0 MSL as placeholder.
    solution.position_ecef_m = core::LlaToEcef(core::Lla{*latitude, *longitude, 0.0});
    solution.fix_type = core::GnssFixType::Single;
    solution.validity = core::MeasurementValidity::Valid;
    return solution;
}

/// @brief Parses a raw RMC line into a GNSS solution.
[[nodiscard]] inline std::optional<core::GnssSolution> ParseRmcLine(std::string_view line) {
    const std::optional<NmeaSentence> sentence = ParseSentence(line);
    if (!sentence) {
        return std::nullopt;
    }
    return ParseRmc(*sentence);
}

/// @brief Writes a FalconGuide RMC sentence from a navigation state.
[[nodiscard]] inline std::string WriteRmc(const core::NavigationState &state, std::string_view utc_time = "",
                                          std::string_view date = "") {
    const core::Lla lla = core::EcefToLla(state.position_ecef_m);

    // Speed over ground: ENU velocity magnitude in the horizontal plane (knots).
    const double speed_enu_mps = std::sqrt(state.velocity_enu_mps.x() * state.velocity_enu_mps.x() +
                                           state.velocity_enu_mps.y() * state.velocity_enu_mps.y());
    constexpr double kMpsToKnots = 1.0 / 0.514444;
    const double speed_knots = speed_enu_mps * kMpsToKnots;

    // Course over ground: angle from North (ENU Y-axis), clockwise, degrees [0,
    // 360).
    double course_deg = core::RadToDeg(std::atan2(state.velocity_enu_mps.x(), state.velocity_enu_mps.y()));
    if (course_deg < 0.0) {
        course_deg += 360.0;
    }

    const std::string lat_str = FormatNmeaLatitudeLongitude(lla.latitude_rad, true);
    const std::string lon_str = FormatNmeaLatitudeLongitude(lla.longitude_rad, false);

    std::ostringstream spd;
    spd << std::fixed << std::setprecision(2) << speed_knots;

    std::ostringstream cog;
    cog << std::fixed << std::setprecision(2) << course_deg;

    const std::vector<std::string> fields{
        std::string(utc_time),
        std::string(1, RmcStatusChar(state.status)),
        lat_str,
        lla.latitude_rad >= 0.0 ? "N" : "S",
        lon_str,
        lla.longitude_rad >= 0.0 ? "E" : "W",
        spd.str(),
        cog.str(),
        std::string(date),
        "",  // magnetic variation — not computed
        "",  // E/W
    };

    return BuildSentence("FG", "RMC", fields);
}

}  // namespace falconguide::io::nmea
