#pragma once

/**
 * @file ui_app.hpp
 * @brief FalconGuide desktop UI — self-contained single binary.
 *
 * UiApp owns both the FalconGuideSession and the WsClient.
 * The session's embedded WsServer listens on a loopback port;
 * the WsClient connects to it within the same process.
 */

#include "falconguide/app/session.hpp"
#include "falconguide/ui/ws_client.hpp"

#include <nlohmann/json.hpp>

#include <array>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

struct GLFWwindow;

namespace falconguide::ui {

struct NavSample {
    double t_s{0.0};
    double lat_deg{0.0};
    double lon_deg{0.0};
    double alt_m{0.0};
    double vel_e{0.0}, vel_n{0.0}, vel_u{0.0};
    double roll_deg{0.0}, pitch_deg{0.0}, yaw_deg{0.0};
    double pos_std_h{0.0}, pos_std_v{0.0};
};

class UiApp {
   public:
    explicit UiApp(uint16_t ws_port = 8765);
    ~UiApp();

    /// @brief Run the main loop (blocks until window is closed).
    void Run();

   private:
    // ── Screens
    void DrawSetupScreen();
    void DrawMainScreen();

    // ── Panels (main screen)
    void DrawStatusBar();
    void DrawTrajectoryPanel();
    void DrawChartPanel();
    void DrawControlPanel();

    // ── Session management
    void StartSession();
    void StopSession();
    void SendCmd(const nlohmann::json& cmd);

    // ── WsClient message handler
    void OnMessage(const nlohmann::json& msg);

    // ── Config
    uint16_t ws_port_;

    // ── Setup form state
    char   dataset_path_buf_[1024]{};
    float  playback_speed_{1.0f};
    int    backend_idx_{0};  // 0=EKF, 1=UKF, 2=Ceres, 3=GTSAM
    char   config_json_buf_[8192]{};
    bool   config_json_dirty_{false};

    // ── Session
    std::unique_ptr<app::FalconGuideSession> session_;

    // ── WebSocket client
    WsClient ws_client_;

    // ── Live state (guarded by mu_)
    mutable std::mutex    mu_;
    std::string           status_{"idle"};
    uint64_t              meas_read_{0};
    bool                  schema_received_{false};
    nlohmann::json        schema_{};

    static constexpr std::size_t kMaxSamples = 3000;
    std::deque<NavSample>        samples_;
    double                       t0_{-1.0};

    // ── GLFW
    GLFWwindow* window_{nullptr};
};

}  // namespace falconguide::ui
