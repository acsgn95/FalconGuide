#include "falconguide/app/session.hpp"
#include "falconguide/estimation/measurement_variant_helpers.hpp"
#include "falconguide/logger/logger.hpp"

#include <chrono>

namespace falconguide::app {

using namespace estimation;

static const char* kLog = "FalconGuideSession";

// ── Constructor / Destructor ──────────────────────────────────────────────────

FalconGuideSession::FalconGuideSession(SessionConfig cfg) : cfg_(std::move(cfg)) {
    // Build nav system
    nav_system_ = std::make_unique<NavigationSystem>(cfg_.nav);
    nav_system_->Pipeline().RegisterObserver(this);

    // Build dataset reader
    dataset_reader_ = MakeDatasetReader(cfg_.dataset);

    // Build IPC server
    ipc_server_ =
        std::make_unique<IpcServer>(cfg_.ipc, [this](const nlohmann::json& cmd, int fd) { HandleCommand(cmd, fd); });

    SetStatus(Status::Configured);
}

FalconGuideSession::~FalconGuideSession() { Stop(); }

// ── Start / Stop / Pause / Resume ─────────────────────────────────────────────

bool FalconGuideSession::Start() {
    const auto s = status_.load();
    if (s == Status::Running || s == Status::Paused) return true;
    if (s == Status::Error) return false;

    if (!dataset_reader_->Open()) {
        SetStatus(Status::Error, "Failed to open dataset: " + cfg_.dataset.path);
        return false;
    }

    if (!ipc_server_->Start()) {
        log::Get()->warn("IPC server failed to start on {}", cfg_.ipc.socket_path);
    }

    nav_system_->Pipeline().Start();
    SetStatus(Status::Running);
    ipc_server_->BroadcastStatus("running");

    running_.store(true);
    replay_thread_ = std::thread(&FalconGuideSession::ReplayLoop, this);

    log::Get()->info("Session started — dataset: {}, socket: {}", cfg_.dataset.path, cfg_.ipc.socket_path);
    return true;
}

void FalconGuideSession::Stop() {
    if (!running_.exchange(false)) return;

    // Wake paused thread
    {
        std::lock_guard lk(pause_mutex_);
        paused_ = false;
    }
    pause_cv_.notify_all();

    if (replay_thread_.joinable()) replay_thread_.join();

    nav_system_->Pipeline().Stop();
    dataset_reader_->Close();
    ipc_server_->BroadcastStatus("idle");
    ipc_server_->Stop();

    SetStatus(Status::Idle);
    log::Get()->info("Session stopped.");
}

void FalconGuideSession::Pause() {
    if (status_.load() != Status::Running) return;
    {
        std::lock_guard lk(pause_mutex_);
        paused_ = true;
    }
    SetStatus(Status::Paused);
    ipc_server_->BroadcastStatus("paused", dataset_reader_->MeasurementsRead());
}

void FalconGuideSession::Resume() {
    if (status_.load() != Status::Paused) return;
    {
        std::lock_guard lk(pause_mutex_);
        paused_ = false;
    }
    pause_cv_.notify_all();
    SetStatus(Status::Running);
    ipc_server_->BroadcastStatus("running", dataset_reader_->MeasurementsRead());
}

bool FalconGuideSession::Reconfigure(SessionConfig cfg) {
    const auto s = status_.load();
    if (s == Status::Running || s == Status::Paused) return false;

    cfg_ = std::move(cfg);
    nav_system_ = std::make_unique<NavigationSystem>(cfg_.nav);
    nav_system_->Pipeline().RegisterObserver(this);
    dataset_reader_ = MakeDatasetReader(cfg_.dataset);
    ipc_server_ =
        std::make_unique<IpcServer>(cfg_.ipc, [this](const nlohmann::json& cmd, int fd) { HandleCommand(cmd, fd); });

    SetStatus(Status::Configured);
    return true;
}

// ── ReplayLoop ────────────────────────────────────────────────────────────────

void FalconGuideSession::ReplayLoop() {
    using Clock = std::chrono::steady_clock;

    std::optional<int64_t> prev_meas_ns;
    auto prev_wall = Clock::now();

    while (running_.load()) {
        // Pause check
        {
            std::unique_lock lk(pause_mutex_);
            pause_cv_.wait(lk, [this] { return !paused_ || !running_.load(); });
        }
        if (!running_.load()) break;

        auto outcome = dataset_reader_->Next();

        if (outcome.result == io::ReadResult::EndOfData) break;
        if (outcome.result != io::ReadResult::Ok || !outcome.measurement) continue;

        // Speed-controlled replay timing
        if (cfg_.playback_speed > 0.0) {
            const auto& ts = estimation::GetTimestamp(*outcome.measurement);
            if (ts.has_steady) {
                const int64_t cur_ns = ts.steady.nanoseconds_since_epoch().count();
                if (prev_meas_ns && cur_ns > *prev_meas_ns) {
                    const int64_t dt_meas_ns = cur_ns - *prev_meas_ns;
                    const auto sleep_ns = static_cast<int64_t>(static_cast<double>(dt_meas_ns) / cfg_.playback_speed);
                    std::this_thread::sleep_until(prev_wall + std::chrono::nanoseconds(sleep_ns));
                }
                prev_meas_ns = cur_ns;
            }
        }
        prev_wall = Clock::now();

        nav_system_->Pipeline().Push(*outcome.measurement);
    }

    if (running_.load()) {
        // Reached end of dataset naturally
        SetStatus(Status::Completed);
        ipc_server_->BroadcastStatus("completed", dataset_reader_->MeasurementsRead());
        log::Get()->info("Dataset replay completed ({} measurements).", dataset_reader_->MeasurementsRead());
    }
}

// ── INavigationObserver ───────────────────────────────────────────────────────

void FalconGuideSession::OnNavigationState(std::shared_ptr<const core::NavigationState> state) {
    if (state) ipc_server_->BroadcastNavState(*state);
}

void FalconGuideSession::OnEstimatorReset() { ipc_server_->BroadcastStatus("reset"); }

// ── IPC command handler ───────────────────────────────────────────────────────

void FalconGuideSession::HandleCommand(const nlohmann::json& cmd, int client_fd) {
    const auto verb = cmd.value("cmd", std::string{});

    if (verb == "start") {
        Start();
    } else if (verb == "stop") {
        Stop();
    } else if (verb == "pause") {
        Pause();
    } else if (verb == "resume") {
        Resume();
    } else if (verb == "get_status") {
        ipc_server_->BroadcastStatus(GetStatusString(), dataset_reader_->MeasurementsRead());
    } else if (verb == "get_schema") {
        ipc_server_->BroadcastSchema(SessionConfig::ToSchemaJson());
    } else if (verb == "set_speed") {
        cfg_.playback_speed = cmd.value("speed", cfg_.playback_speed);
    } else if (verb == "configure") {
        if (cmd.contains("config")) {
            try {
                auto new_cfg = SessionConfig::FromJson(cmd["config"].dump());
                Reconfigure(std::move(new_cfg));
                ipc_server_->BroadcastStatus("configured");
            } catch (const std::exception& ex) {
                ipc_server_->BroadcastError(std::string("configure failed: ") + ex.what());
            }
        }
    } else {
        ipc_server_->BroadcastError("unknown command: " + verb);
    }
}

// ── Status helpers ────────────────────────────────────────────────────────────

void FalconGuideSession::SetStatus(Status s, const std::string& error) {
    status_.store(s);
    if (!error.empty()) {
        std::lock_guard lk(error_mutex_);
        error_ = error;
        log::Get()->error("{}", error);
        ipc_server_->BroadcastError(error);
    }
}

FalconGuideSession::Status FalconGuideSession::GetStatus() const { return status_.load(); }

std::string FalconGuideSession::GetStatusString() const {
    switch (status_.load()) {
        case Status::Idle:
            return "idle";
        case Status::Configured:
            return "configured";
        case Status::Running:
            return "running";
        case Status::Paused:
            return "paused";
        case Status::Completed:
            return "completed";
        case Status::Error:
            return "error";
    }
    return "unknown";
}

std::string FalconGuideSession::GetError() const {
    std::lock_guard lk(error_mutex_);
    return error_;
}

}  // namespace falconguide::app
