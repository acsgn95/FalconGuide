#pragma once

/**
 * @file logger_config.hpp
 * @brief Logger sink, format, and level configuration.
 */

#include <cstdint>
#include <optional>
#include <string>

namespace falconguide::log {

// ── Log levels
// ────────────────────────────────────────────────────────────────
enum class Level {
    Trace,     ///< Most verbose diagnostic logging.
    Debug,     ///< Debug-level diagnostics.
    Info,      ///< Informational messages.
    Warn,      ///< Warning messages.
    Error,     ///< Error messages.
    Critical,  ///< Critical/fatal messages.
    Off        ///< Logging disabled.
};

// ── NetworkLogConfig
// ──────────────────────────────────────────────────────────
//
// Streams log messages as UDP datagrams to a remote host (e.g. ground station).
// Each datagram is one formatted log line; no framing protocol is added.
//
struct NetworkLogConfig {
    std::string host{"127.0.0.1"};  ///< Destination hostname or IP address.
    std::uint16_t port{5140};       ///< Destination UDP port.
};

// ── LoggerConfig
// ──────────────────────────────────────────────────────────────
struct LoggerConfig {
    std::string name{"falconguide"};  ///< Logger instance name shown in each line.

    Level level{Level::Info};  ///< Runtime logging level.

    // ── Console sink ──────────────────────────────────────────────────────────
    bool console{true};  ///< Enables console sink.

    // ── File sink ─────────────────────────────────────────────────────────────
    // Rotating file: when the log reaches max_file_size_mb it is rotated and
    // up to max_files old logs are kept.
    std::optional<std::string> file_path;  ///< Optional rotating log file path.
    std::size_t max_file_size_mb{10};      ///< Rotation threshold in MiB.
    std::size_t max_files{5};              ///< Number of rotated files to retain.

    // ── Network sink (UDP) ────────────────────────────────────────────────────
    std::optional<NetworkLogConfig> network;  ///< Optional UDP network logging sink.

    // ── Format ────────────────────────────────────────────────────────────────
    // spdlog pattern string.  Default includes timestamp, name, level, thread id.
    std::string pattern{"%^[%Y-%m-%d %H:%M:%S.%e] [%n] [%-8l] [t:%t]%$ %v"};  ///< spdlog pattern.
};

}  // namespace falconguide::log
