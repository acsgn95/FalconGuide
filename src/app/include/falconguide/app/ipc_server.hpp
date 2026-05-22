#pragma once

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

// ── IpcServer ─────────────────────────────────────────────────────────────────
//
// Unix domain socket server.  One server thread accepts connections and
// dispatches commands.  Broadcast() is thread-safe and sends to all clients.
//
class IpcServer {
 public:
  using CommandHandler = std::function<void(const nlohmann::json& cmd, int client_fd)>;

  explicit IpcServer(IpcConfig cfg, CommandHandler handler);
  ~IpcServer();

  // Non-copyable, non-movable
  IpcServer(const IpcServer&) = delete;
  IpcServer& operator=(const IpcServer&) = delete;

  bool Start();
  void Stop();
  [[nodiscard]] bool IsRunning() const { return running_.load(); }

  // Thread-safe: broadcast JSON event to all connected clients
  void Broadcast(const nlohmann::json& event);

  // Convenience builders
  void BroadcastNavState(const core::NavigationState& state);
  void BroadcastStatus(const std::string& status, std::size_t measurements_read = 0);
  void BroadcastError(const std::string& message);
  void BroadcastSchema(const nlohmann::json& schema);

 private:
  void ServerLoop();
  void HandleData(int fd, const std::string& line);
  void RemoveClient(int fd);
  void WriteToClient(int fd, const std::string& msg);

  IpcConfig     cfg_;
  CommandHandler handler_;
  int           listen_fd_{-1};
  std::atomic<bool> running_{false};
  std::thread   thread_;
  std::mutex    clients_mutex_;
  std::vector<int> clients_;
};

}  // namespace falconguide::app
