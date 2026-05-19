#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace falconguide::log {

// ── Log levels ────────────────────────────────────────────────────────────────
enum class Level {
  Trace,
  Debug,
  Info,
  Warn,
  Error,
  Critical,
  Off
};

// ── NetworkLogConfig ──────────────────────────────────────────────────────────
//
// Streams log messages as UDP datagrams to a remote host (e.g. ground station).
// Each datagram is one formatted log line; no framing protocol is added.
//
struct NetworkLogConfig {
  std::string host{"127.0.0.1"};
  std::uint16_t port{5140};
};

// ── LoggerConfig ──────────────────────────────────────────────────────────────
struct LoggerConfig {
  // Logger instance name — appears in every line.
  std::string name{"falconguide"};

  Level level{Level::Info};

  // ── Console sink ──────────────────────────────────────────────────────────
  bool console{true};

  // ── File sink ─────────────────────────────────────────────────────────────
  // Rotating file: when the log reaches max_file_size_mb it is rotated and
  // up to max_files old logs are kept.
  std::optional<std::string> file_path;       // e.g. "/var/log/falconguide/nav.log"
  std::size_t max_file_size_mb{10};
  std::size_t max_files{5};

  // ── Network sink (UDP) ────────────────────────────────────────────────────
  std::optional<NetworkLogConfig> network;

  // ── Format ────────────────────────────────────────────────────────────────
  // spdlog pattern string.  Default includes timestamp, name, level, thread id.
  std::string pattern{"%^[%Y-%m-%d %H:%M:%S.%e] [%n] [%-8l] [t:%t]%$ %v"};
};

}  // namespace falconguide::log
