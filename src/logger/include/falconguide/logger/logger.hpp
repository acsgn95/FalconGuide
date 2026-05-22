#pragma once

#include "falconguide/logger/logger_config.hpp"

// Pull in spdlog's source-location macros so FG_* macros capture file/line.
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#include <spdlog/spdlog.h>

#include <memory>
#include <string_view>

namespace falconguide::log {

// ── Lifecycle ─────────────────────────────────────────────────────────────────

// Call once at program start.  Safe to call multiple times (re-initialises).
void Init(const LoggerConfig& config = {});

// Flush all sinks and release resources.  Call at program exit.
void Shutdown();

// ── Access ────────────────────────────────────────────────────────────────────

// Returns the shared logger instance.  Asserts if Init() was not called.
[[nodiscard]] std::shared_ptr<spdlog::logger> Get();

// Change runtime log level without re-initialising.
void SetLevel(Level level);

}  // namespace falconguide::log

// ── Logging macros ────────────────────────────────────────────────────────────
//
// Use these throughout FalconGuide instead of calling spdlog directly.
// They capture __FILE__ and __LINE__ automatically and route through the
// shared falconguide logger instance.
//
// Example:
//   FG_INFO("EKF accepted, model={}, correction={:.4f}m", report.model_name, *report.correction_norm);
//   FG_WARN("Dead reckoning for {:.1f}s — no aiding", age_s);
//   FG_ERROR("Covariance ill-conditioned, regularising");
//
#define FG_TRACE(...)    SPDLOG_LOGGER_TRACE(   ::falconguide::log::Get(), __VA_ARGS__)
#define FG_DEBUG(...)    SPDLOG_LOGGER_DEBUG(   ::falconguide::log::Get(), __VA_ARGS__)
#define FG_INFO(...)     SPDLOG_LOGGER_INFO(    ::falconguide::log::Get(), __VA_ARGS__)
#define FG_WARN(...)     SPDLOG_LOGGER_WARN(    ::falconguide::log::Get(), __VA_ARGS__)
#define FG_ERROR(...)    SPDLOG_LOGGER_ERROR(   ::falconguide::log::Get(), __VA_ARGS__)
#define FG_CRITICAL(...) SPDLOG_LOGGER_CRITICAL(::falconguide::log::Get(), __VA_ARGS__)
