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
#include "falconguide/ui/tile_map.hpp"
#include "falconguide/ui/ws_client.hpp"
#include "falconguide/vo/vo_processor.hpp"

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
    void DrawCameraPanel();

    // ── Session management
    void StartSession();
    void StopSession();
    void SendCmd(const nlohmann::json& cmd);

    // ── WsClient message handler
    void OnMessage(const nlohmann::json& msg);

    // ── Config
    uint16_t ws_port_;

    // ── Setup form state
    char dataset_path_buf_[1024]{};
    int dataset_type_idx_{0};  // 0=sensor_logger, 1=csv
    float playback_speed_{1.0f};
    int backend_idx_{0};  // 0=EKF, 1=UKF, 2=Ceres, 3=GTSAM
    char config_json_buf_[8192]{};
    bool config_json_dirty_{false};

    // ── Session
    std::unique_ptr<app::FalconGuideSession> session_;

    // ── WebSocket client
    WsClient ws_client_;

    // ── Live state (guarded by mu_)
    mutable std::mutex mu_;
    std::string status_{"idle"};
    uint64_t meas_read_{0};
    bool schema_received_{false};
    nlohmann::json schema_{};

    static constexpr std::size_t kMaxSamples = 3000;
    std::deque<NavSample> samples_;
    double t0_{-1.0};

    // ── Map
    TileMap tile_map_;

    // ── VO processor + camera state
    vo::VoProcessor vo_processor_;
    std::string pending_camera_path_;  ///< Set by OnMessage, consumed by DrawCameraPanel
    unsigned int camera_texture_{0};   ///< OpenGL texture for current frame
    int camera_tex_w_{0};
    int camera_tex_h_{0};
    vo::VoResult last_vo_result_;
    std::vector<vo::VoPose> vo_trajectory_snapshot_;

    // ── GLFW
    GLFWwindow* window_{nullptr};
};

}  // namespace falconguide::ui
