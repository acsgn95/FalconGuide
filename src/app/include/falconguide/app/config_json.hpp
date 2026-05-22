#pragma once

#include "falconguide/estimation/navigation_system_config.hpp"

#include <nlohmann/json.hpp>

#include <string>

namespace falconguide::app {

// ── Dataset config ────────────────────────────────────────────────────────────

struct DatasetConfig {
  std::string type{"csv"};   // "csv" now; "ros_bag", "mcap" later
  std::string path;
};

// ── IPC config ────────────────────────────────────────────────────────────────

struct IpcConfig {
  std::string socket_path{"/tmp/falconguide.sock"};
  int         max_clients{8};
};

// ── Session config (top-level) ────────────────────────────────────────────────

struct SessionConfig {
  estimation::NavigationSystemConfig nav;
  DatasetConfig                      dataset;
  IpcConfig                          ipc;
  double                             playback_speed{1.0};

  static SessionConfig FromJson(const std::string& json_str);
  static SessionConfig FromPath(const std::string& path);

  [[nodiscard]] std::string        ToJson()   const;
  void                             SaveToPath(const std::string& path) const;
  [[nodiscard]] static nlohmann::json ToSchemaJson();
  [[nodiscard]] bool               Validate(std::string& error) const;
};

}  // namespace falconguide::app
