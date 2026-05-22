#pragma once

/**
 * @file ipc_server.hpp
 * @brief Unix-domain-socket IPC server for session control and telemetry.
 */

#include "falconguide/app/config_json.hpp"
#include "falconguide/core/navigation_state.hpp"

#include <nlohmann/json.hpp>

#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace falconguide::app {

// ── IpcServer
// ─────────────────────────────────────────────────────────────────
//
// Unix domain socket server.  One server thread accepts connections and
// dispatches commands.  Broadcast() is thread-safe and sends to all clients.
//
class IpcServer {
public:
  /// @brief Callback invoked for each decoded command.
  using CommandHandler =
      std::function<void(const nlohmann::json &cmd, int client_fd)>;

  /// @brief Constructs an IPC server.
  explicit IpcServer(IpcConfig cfg, CommandHandler handler);
  /// @brief Stops the server and closes client connections.
  ~IpcServer();

  // Non-copyable, non-movable
  IpcServer(const IpcServer &) = delete;
  IpcServer &operator=(const IpcServer &) = delete;

  /// @brief Starts the server thread and begins accepting clients.
  bool Start();
  /// @brief Stops the server thread and closes connected clients.
  void Stop();
  /// @brief Returns true while the server thread is running.
  [[nodiscard]] bool IsRunning() const { return running_.load(); }

  /// @brief Broadcasts a JSON event to all connected clients.
  void Broadcast(const nlohmann::json &event);

  /// @brief Broadcasts a navigation-state event.
  void BroadcastNavState(const core::NavigationState &state);
  /// @brief Broadcasts a status event.
  void BroadcastStatus(const std::string &status,
                       std::size_t measurements_read = 0);
  /// @brief Broadcasts an error event.
  void BroadcastError(const std::string &message);
  /// @brief Broadcasts a schema event.
  void BroadcastSchema(const nlohmann::json &schema);

private:
  void ServerLoop();
  void HandleData(int fd, const std::string &line);
  void RemoveClient(int fd);
  void WriteToClient(int fd, const std::string &msg);

  IpcConfig cfg_;
  CommandHandler handler_;
  int listen_fd_{-1};
  std::atomic<bool> running_{false};
  std::thread thread_;
  std::mutex clients_mutex_;
  std::vector<int> clients_;
};

} // namespace falconguide::app
