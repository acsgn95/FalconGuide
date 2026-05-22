#pragma once

#include "falconguide/app/config_json.hpp"
#include "falconguide/app/dataset_reader.hpp"
#include "falconguide/app/ipc_server.hpp"
#include "falconguide/estimation/navigation_observer.hpp"
#include "falconguide/estimation/navigation_system.hpp"

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace falconguide::app {

// ── FalconGuideSession ────────────────────────────────────────────────────────
//
// Top-level lifecycle object.  Owns the NavigationSystem, dataset reader, and
// IPC server.  A replay thread feeds measurements into the pipeline at the
// configured playback speed.
//
// State machine:
//   Idle → Configured → Running ⇄ Paused → Completed
//                                         ↘ Error
//
// Typical use (programmatic):
//   auto cfg = SessionConfig::FromPath("config.json");
//   FalconGuideSession session(cfg);
//   session.Start();
//   ...
//   session.Stop();
//
// With IPC:  the CLI sets up the session then blocks; the IPC client drives
// Start/Stop/Pause/Resume via JSON commands.
//
class FalconGuideSession : private estimation::INavigationObserver {
 public:
  enum class Status { Idle, Configured, Running, Paused, Completed, Error };

  explicit FalconGuideSession(SessionConfig cfg);
  ~FalconGuideSession();

  // Non-copyable
  FalconGuideSession(const FalconGuideSession&) = delete;
  FalconGuideSession& operator=(const FalconGuideSession&) = delete;

  // Lifecycle
  bool Start();      // open dataset, start pipeline + IPC
  void Stop();       // graceful stop
  void Pause();
  void Resume();

  // Reconfigure (only when Idle or after Stop)
  bool Reconfigure(SessionConfig cfg);

  // Status
  [[nodiscard]] Status      GetStatus()      const;
  [[nodiscard]] std::string GetStatusString() const;
  [[nodiscard]] std::string GetError()       const;
  [[nodiscard]] bool        IsRunning()      const { return status_.load() == Status::Running; }

  // Direct access when needed (e.g. additional observers)
  estimation::NavigationSystem& NavSystem() { return *nav_system_; }

 private:
  // INavigationObserver
  void OnNavigationState(std::shared_ptr<const core::NavigationState> state) override;
  void OnEstimatorReset() override;

  void ReplayLoop();
  void SetStatus(Status s, const std::string& error = {});
  void HandleCommand(const nlohmann::json& cmd, int client_fd);

  SessionConfig cfg_;

  std::unique_ptr<estimation::NavigationSystem> nav_system_;
  std::unique_ptr<IDatasetReader>               dataset_reader_;
  std::unique_ptr<IpcServer>                    ipc_server_;

  std::atomic<Status> status_{Status::Idle};
  std::string         error_;
  mutable std::mutex  error_mutex_;

  std::thread    replay_thread_;
  std::atomic<bool> running_{false};

  bool           paused_{false};
  std::mutex     pause_mutex_;
  std::condition_variable pause_cv_;
};

}  // namespace falconguide::app
