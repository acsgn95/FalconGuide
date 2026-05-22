#include "falconguide/app/session.hpp"
#include "falconguide/logger/logger.hpp"

#include <csignal>
#include <cstdlib>
#include <iostream>
#include <thread>

// ── Signal handling ───────────────────────────────────────────────────────────

static falconguide::app::FalconGuideSession* g_session = nullptr;

static void OnSignal(int) {
    if (g_session) g_session->Stop();
}

// ── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    // ── Usage ─────────────────────────────────────────────────────────────────
    if (argc < 2) {
        std::cerr << "Usage: falconguide_cli <config.json> [--print-schema]\n";
        return EXIT_FAILURE;
    }

    // ── Print schema mode ─────────────────────────────────────────────────────
    if (argc >= 3 && std::string(argv[2]) == "--print-schema") {
        std::cout << falconguide::app::SessionConfig::ToSchemaJson().dump(2) << "\n";
        return EXIT_SUCCESS;
    }

    // ── Logger ────────────────────────────────────────────────────────────────
    falconguide::log::LoggerConfig log_cfg;
    log_cfg.console = true;
    falconguide::log::Init(log_cfg);

    const auto log = falconguide::log::Get();
    log->info("FalconGuide CLI — config: {}", argv[1]);

    // ── Load config ───────────────────────────────────────────────────────────
    falconguide::app::SessionConfig cfg;
    try {
        cfg = falconguide::app::SessionConfig::FromPath(argv[1]);
    } catch (const std::exception& ex) {
        log->error("Failed to load config: {}", ex.what());
        return EXIT_FAILURE;
    }

    std::string validation_error;
    if (!cfg.Validate(validation_error)) {
        log->error("Config validation failed: {}", validation_error);
        return EXIT_FAILURE;
    }

    // ── Create session ────────────────────────────────────────────────────────
    falconguide::app::FalconGuideSession session(cfg);
    g_session = &session;

    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    log->info("IPC socket: {}", cfg.ipc.socket_path);
    log->info("Dataset:    {} ({})", cfg.dataset.path, cfg.dataset.type);
    log->info("Backend:    {}",
              cfg.nav.backend == falconguide::estimation::EstimatorBackendChoice::Ukf ? "UKF" : "EKF");
    log->info("Playback:   {}x speed", cfg.playback_speed == 0.0 ? "max" : std::to_string(cfg.playback_speed));

    // ── Start ─────────────────────────────────────────────────────────────────
    if (!session.Start()) {
        log->error("Session failed to start: {}", session.GetError());
        return EXIT_FAILURE;
    }

    // ── Wait until done ───────────────────────────────────────────────────────
    while (true) {
        const auto s = session.GetStatus();
        if (s == falconguide::app::FalconGuideSession::Status::Completed ||
            s == falconguide::app::FalconGuideSession::Status::Idle ||
            s == falconguide::app::FalconGuideSession::Status::Error) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    session.Stop();
    falconguide::log::Shutdown();
    return EXIT_SUCCESS;
}
