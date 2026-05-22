#include "falconguide/logger/logger.hpp"

#include <spdlog/async.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/udp_sink.h>

#include <cassert>
#include <vector>

namespace falconguide::log {

namespace {

std::shared_ptr<spdlog::logger> g_logger;

spdlog::level::level_enum ToSpdlogLevel(Level level) {
  switch (level) {
    case Level::Trace:    return spdlog::level::trace;
    case Level::Debug:    return spdlog::level::debug;
    case Level::Info:     return spdlog::level::info;
    case Level::Warn:     return spdlog::level::warn;
    case Level::Error:    return spdlog::level::err;
    case Level::Critical: return spdlog::level::critical;
    case Level::Off:      return spdlog::level::off;
  }
  return spdlog::level::info;
}

}  // namespace

void Init(const LoggerConfig& config) {
  std::vector<spdlog::sink_ptr> sinks;

  // ── Console sink (colored) ────────────────────────────────────────────────
  if (config.console) {
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_pattern(config.pattern);
    sinks.push_back(std::move(console_sink));
  }

  // ── Rotating file sink ────────────────────────────────────────────────────
  if (config.file_path) {
    const std::size_t max_bytes = config.max_file_size_mb * 1024 * 1024;
    auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        *config.file_path, max_bytes, config.max_files);
    // File sink: plain pattern (no ANSI colours in log files)
    const std::string file_pattern =
        "[%Y-%m-%d %H:%M:%S.%e] [" + config.name + "] [%-8l] [t:%t] %v";
    file_sink->set_pattern(file_pattern);
    sinks.push_back(std::move(file_sink));
  }

  // ── UDP network sink ──────────────────────────────────────────────────────
  if (config.network) {
    spdlog::sinks::udp_sink_config udp_cfg{config.network->host, config.network->port};
    auto udp_sink = std::make_shared<spdlog::sinks::udp_sink_mt>(udp_cfg);
    const std::string net_pattern =
        "[%Y-%m-%d %H:%M:%S.%e] [" + config.name + "] [%-8l] [t:%t] %v";
    udp_sink->set_pattern(net_pattern);
    sinks.push_back(std::move(udp_sink));
  }

  // ── Assemble logger ───────────────────────────────────────────────────────
  g_logger = std::make_shared<spdlog::logger>(config.name, sinks.begin(), sinks.end());
  g_logger->set_level(ToSpdlogLevel(config.level));

  // Flush on warnings and above so critical events are never lost in a buffer.
  g_logger->flush_on(spdlog::level::warn);

  spdlog::register_logger(g_logger);
}

void Shutdown() {
  if (g_logger) {
    g_logger->flush();
  }
  spdlog::shutdown();
  g_logger.reset();
}

std::shared_ptr<spdlog::logger> Get() {
  assert(g_logger && "falconguide::log::Init() was not called");
  return g_logger;
}

void SetLevel(Level level) {
  if (g_logger) {
    g_logger->set_level(ToSpdlogLevel(level));
  }
}

}  // namespace falconguide::log
