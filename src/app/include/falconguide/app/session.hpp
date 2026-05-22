#pragma once

/**
 * @file session.hpp
 * @brief Top-level FalconGuide session lifecycle and replay orchestration.
 */

#include "falconguide/app/config_json.hpp"
#include "falconguide/app/dataset_reader.hpp"
#include "falconguide/app/ipc_server.hpp"
#include "falconguide/app/ws_server.hpp"
#include "falconguide/estimation/navigation_observer.hpp"
#include "falconguide/estimation/navigation_system.hpp"

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace falconguide::app {

// ── FalconGuideSession
// ────────────────────────────────────────────────────────
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
    /// @brief Session state-machine states.
    enum class Status {
        Idle,        ///< No active configuration or running work.
        Configured,  ///< Session is configured and ready.
        Running,     ///< Replay and pipeline are active.
        Paused,      ///< Replay is paused.
        Completed,   ///< Dataset replay completed.
        Error        ///< Session encountered an unrecoverable error.
    };

    /// @brief Constructs a session from configuration.
    explicit FalconGuideSession(SessionConfig cfg);
    /// @brief Stops owned threads and releases resources.
    ~FalconGuideSession();

    // Non-copyable
    FalconGuideSession(const FalconGuideSession &) = delete;
    FalconGuideSession &operator=(const FalconGuideSession &) = delete;

    /// @brief Opens dataset, starts pipeline, and starts IPC.
    bool Start();
    /// @brief Gracefully stops replay, pipeline, and IPC.
    void Stop();
    /// @brief Pauses replay.
    void Pause();
    /// @brief Resumes replay after Pause().
    void Resume();

    /// @brief Reconfigures the session when idle or stopped.
    bool Reconfigure(SessionConfig cfg);

    /// @brief Returns current session status.
    [[nodiscard]] Status GetStatus() const;
    /// @brief Returns current session status as text.
    [[nodiscard]] std::string GetStatusString() const;
    /// @brief Returns the latest error string.
    [[nodiscard]] std::string GetError() const;
    /// @brief Returns true when the session is running.
    [[nodiscard]] bool IsRunning() const { return status_.load() == Status::Running; }

    /// @brief Returns the owned navigation system.
    estimation::NavigationSystem &NavSystem() { return *nav_system_; }

   private:
    // INavigationObserver
    void OnNavigationState(std::shared_ptr<const core::NavigationState> state) override;
    void OnEstimatorReset() override;

    void ReplayLoop();
    void SetStatus(Status s, const std::string &error = {});
    void HandleCommand(const nlohmann::json &cmd, int client_fd);
    nlohmann::json HandleWsCommand(const nlohmann::json &cmd);

    SessionConfig cfg_;

    std::unique_ptr<estimation::NavigationSystem> nav_system_;
    std::unique_ptr<IDatasetReader> dataset_reader_;
    std::unique_ptr<IpcServer> ipc_server_;
    std::unique_ptr<WsServer> ws_server_;  // optional, null when port == 0

    std::atomic<Status> status_{Status::Idle};
    std::string error_;
    mutable std::mutex error_mutex_;

    std::thread replay_thread_;
    std::atomic<bool> running_{false};

    bool paused_{false};
    std::mutex pause_mutex_;
    std::condition_variable pause_cv_;
};

}  // namespace falconguide::app
