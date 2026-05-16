#pragma once

#include <cstdint>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace falconguide::io::nmea {

struct NmeaSentence {
  std::string talker;
  std::string formatter;
  std::vector<std::string> fields;
};

[[nodiscard]] inline std::uint8_t ComputeChecksum(std::string_view payload) {
  std::uint8_t checksum = 0;
  for (const char character : payload) {
    checksum ^= static_cast<std::uint8_t>(character);
  }
  return checksum;
}

[[nodiscard]] inline std::string FormatChecksum(std::uint8_t checksum) {
  std::ostringstream stream;
  stream << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(checksum);
  return stream.str();
}

[[nodiscard]] inline std::optional<std::uint8_t> ParseHexByte(std::string_view text) {
  if (text.size() != 2) {
    return std::nullopt;
  }

  std::uint8_t value = 0;
  for (const char character : text) {
    value <<= 4;
    if (character >= '0' && character <= '9') {
      value |= static_cast<std::uint8_t>(character - '0');
    } else if (character >= 'A' && character <= 'F') {
      value |= static_cast<std::uint8_t>(character - 'A' + 10);
    } else if (character >= 'a' && character <= 'f') {
      value |= static_cast<std::uint8_t>(character - 'a' + 10);
    } else {
      return std::nullopt;
    }
  }
  return value;
}

[[nodiscard]] inline std::optional<NmeaSentence> ParseSentence(std::string_view line) {
  if (line.size() < 9 || line.front() != '$') {
    return std::nullopt;
  }

  while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) {
    line.remove_suffix(1);
  }

  const std::size_t checksum_marker = line.find('*');
  if (checksum_marker == std::string_view::npos || checksum_marker + 3 != line.size()) {
    return std::nullopt;
  }

  const std::string_view payload = line.substr(1, checksum_marker - 1);
  const std::optional<std::uint8_t> expected_checksum = ParseHexByte(line.substr(checksum_marker + 1, 2));
  if (!expected_checksum || ComputeChecksum(payload) != *expected_checksum) {
    return std::nullopt;
  }

  const std::size_t first_comma = payload.find(',');
  const std::string_view header = first_comma == std::string_view::npos ? payload : payload.substr(0, first_comma);
  if (header.size() < 5) {
    return std::nullopt;
  }

  NmeaSentence sentence;
  sentence.talker = std::string(header.substr(0, 2));
  sentence.formatter = std::string(header.substr(2));

  if (first_comma == std::string_view::npos) {
    return sentence;
  }

  std::size_t field_start = first_comma + 1;
  while (field_start <= payload.size()) {
    const std::size_t field_end = payload.find(',', field_start);
    if (field_end == std::string_view::npos) {
      sentence.fields.emplace_back(payload.substr(field_start));
      break;
    }
    sentence.fields.emplace_back(payload.substr(field_start, field_end - field_start));
    field_start = field_end + 1;
  }

  return sentence;
}

[[nodiscard]] inline std::string BuildSentence(std::string_view talker, std::string_view formatter,
                                               const std::vector<std::string>& fields) {
  std::ostringstream payload;
  payload << talker << formatter;
  for (const std::string& field : fields) {
    payload << ',' << field;
  }

  const std::string payload_text = payload.str();
  return "$" + payload_text + "*" + FormatChecksum(ComputeChecksum(payload_text));
}

}  // namespace falconguide::io::nmea
