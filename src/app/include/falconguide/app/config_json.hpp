#pragma once

/**
 * @file config_json.hpp
 * @brief JSON serialization, validation, and schema helpers for session
 * configuration.
 */

#include "falconguide/estimation/navigation_system_config.hpp"

#include <nlohmann/json.hpp>

#include <string>

namespace falconguide::app {

// ── Dataset config
// ────────────────────────────────────────────────────────────

struct DatasetConfig {
  std::string type{"csv"}; ///< Dataset type, currently "csv".
  std::string path;        ///< Dataset path.
};

// ── IPC config
// ────────────────────────────────────────────────────────────────

struct IpcConfig {
  std::string socket_path{
      "/tmp/falconguide.sock"}; ///< Unix domain socket path.
  int max_clients{8};           ///< Maximum connected clients.
};

// ── Session config (top-level)
// ────────────────────────────────────────────────

struct SessionConfig {
  estimation::NavigationSystemConfig nav; ///< Navigation-system configuration.
  DatasetConfig dataset;                  ///< Dataset reader configuration.
  IpcConfig ipc;                          ///< IPC server configuration.
  double playback_speed{1.0};             ///< Replay speed multiplier.

  /// @brief Parses a session configuration from JSON text.
  static SessionConfig FromJson(const std::string &json_str);
  /// @brief Loads and parses a session configuration from a file.
  static SessionConfig FromPath(const std::string &path);

  /// @brief Serializes the configuration to JSON text.
  [[nodiscard]] std::string ToJson() const;
  /// @brief Writes the configuration to a file path.
  void SaveToPath(const std::string &path) const;
  /// @brief Returns a JSON schema for supported session configuration.
  [[nodiscard]] static nlohmann::json ToSchemaJson();
  /// @brief Validates semantic configuration constraints.
  [[nodiscard]] bool Validate(std::string &error) const;
};

} // namespace falconguide::app
