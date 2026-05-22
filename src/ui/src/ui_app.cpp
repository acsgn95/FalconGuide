#include "falconguide/ui/ui_app.hpp"

#include "falconguide/app/config_json.hpp"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace falconguide::ui {

// ── Helpers ───────────────────────────────────────────────────────────────────

static void glfw_error_cb(int err, const char* msg) {
    std::fprintf(stderr, "GLFW %d: %s\n", err, msg);
}

template <typename F>
static std::vector<double> col(const std::deque<NavSample>& s, F fn) {
    std::vector<double> v;
    v.reserve(s.size());
    for (auto& x : s) v.push_back(fn(x));
    return v;
}

// ── Constructor / Destructor ──────────────────────────────────────────────────

UiApp::UiApp(uint16_t ws_port)
    : ws_port_(ws_port),
      ws_client_("127.0.0.1", ws_port, [this](const nlohmann::json& m) { OnMessage(m); }) {}

UiApp::~UiApp() {
    StopSession();
    if (window_) {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImPlot::DestroyContext();
        ImGui::DestroyContext();
        glfwDestroyWindow(window_);
        glfwTerminate();
    }
}

// ── Session management ────────────────────────────────────────────────────────

void UiApp::StartSession() {
    StopSession();

    app::SessionConfig cfg;
    cfg.dataset.path    = dataset_path_buf_;
    cfg.playback_speed  = double(playback_speed_);
    cfg.ws.port         = ws_port_;
    cfg.ws.max_clients  = 4;

    static const char* kBackends[] = {"ekf", "ukf", "ceres", "gtsam"};
    // Parse override JSON if the user edited it
    if (config_json_dirty_ && config_json_buf_[0] != '\0') {
        try {
            cfg = app::SessionConfig::FromJson(config_json_buf_);
            cfg.dataset.path   = dataset_path_buf_;  // path always from picker
            cfg.ws.port        = ws_port_;
        } catch (...) { /* fallback to defaults */ }
    }

    // Apply backend choice from dropdown (unless overridden by JSON)
    if (!config_json_dirty_) {
        (void)kBackends[backend_idx_]; // nav backend wired via NavigationSystemConfig
        // For now backend selection is a nav config field — use defaults
    }

    try {
        session_ = std::make_unique<app::FalconGuideSession>(cfg);
        session_->Start();
        // Give WsServer a moment to bind before the client connects
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        ws_client_.Connect();
    } catch (const std::exception& ex) {
        std::fprintf(stderr, "Session start failed: %s\n", ex.what());
        session_.reset();
    }
}

void UiApp::StopSession() {
    ws_client_.Disconnect();
    if (session_) {
        session_->Stop();
        session_.reset();
    }
    std::lock_guard lk(mu_);
    samples_.clear();
    t0_ = -1.0;
    status_ = "idle";
    meas_read_ = 0;
}

void UiApp::SendCmd(const nlohmann::json& cmd) { ws_client_.Send(cmd); }

// ── WsClient message handler ──────────────────────────────────────────────────

void UiApp::OnMessage(const nlohmann::json& msg) {
    std::lock_guard lk(mu_);
    const auto ev = msg.value("event", std::string{});

    if (ev == "nav_state") {
        NavSample s;
        s.lat_deg   = msg.value("lat_deg",    0.0);
        s.lon_deg   = msg.value("lon_deg",    0.0);
        s.alt_m     = msg.value("alt_m",      0.0);
        s.pos_std_h = msg.value("pos_std_h_m",0.0);
        s.pos_std_v = msg.value("pos_std_v_m",0.0);
        s.roll_deg  = msg.value("roll_deg",   0.0);
        s.pitch_deg = msg.value("pitch_deg",  0.0);
        s.yaw_deg   = msg.value("yaw_deg",    0.0);
        if (msg.contains("vel_enu_mps") && msg["vel_enu_mps"].is_array()) {
            s.vel_e = msg["vel_enu_mps"][0].get<double>();
            s.vel_n = msg["vel_enu_mps"][1].get<double>();
            s.vel_u = msg["vel_enu_mps"][2].get<double>();
        }
        s.t_s = msg.value("timestamp_ns", int64_t{0}) * 1e-9;
        if (t0_ < 0.0) t0_ = s.t_s;
        s.t_s -= t0_;
        samples_.push_back(s);
        if (samples_.size() > kMaxSamples) samples_.pop_front();
    } else if (ev == "status") {
        status_    = msg.value("status", status_);
        meas_read_ = msg.value("measurements_read", meas_read_);
    } else if (ev == "schema") {
        schema_          = msg.value("schema", nlohmann::json{});
        schema_received_ = true;
    } else if (ev == "error") {
        status_ = "error: " + msg.value("message", "");
    }
}

// ── Run ───────────────────────────────────────────────────────────────────────

void UiApp::Run() {
    glfwSetErrorCallback(glfw_error_cb);
    if (!glfwInit()) return;

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    window_ = glfwCreateWindow(1400, 900, "FalconGuide", nullptr, nullptr);
    if (!window_) { glfwTerminate(); return; }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGui::StyleColorsDark();
    auto& style = ImGui::GetStyle();
    style.WindowRounding = 4.0f;
    style.FrameRounding  = 3.0f;
    style.GrabRounding   = 3.0f;

    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    while (!glfwWindowShouldClose(window_)) {
        glfwPollEvents();
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        if (!session_)
            DrawSetupScreen();
        else
            DrawMainScreen();

        ImGui::Render();
        int fw, fh;
        glfwGetFramebufferSize(window_, &fw, &fh);
        glViewport(0, 0, fw, fh);
        glClearColor(0.08f, 0.08f, 0.10f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window_);
    }
}

// ── Setup screen ──────────────────────────────────────────────────────────────

void UiApp::DrawSetupScreen() {
    int fw, fh;
    glfwGetFramebufferSize(window_, &fw, &fh);

    const float w = 560.0f, h = 340.0f;
    ImGui::SetNextWindowPos({(fw - w) * 0.5f, (fh - h) * 0.5f}, ImGuiCond_Always);
    ImGui::SetNextWindowSize({w, h}, ImGuiCond_Always);
    ImGui::Begin("##setup", nullptr,
                 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                 ImGuiWindowFlags_NoMove     | ImGuiWindowFlags_NoCollapse);

    // Title
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.4f, 0.8f, 1.0f, 1.0f));
    ImGui::SetWindowFontScale(1.4f);
    ImGui::Text("FalconGuide");
    ImGui::SetWindowFontScale(1.0f);
    ImGui::PopStyleColor();
    ImGui::TextDisabled("Visual-Inertial GNSS Navigation System");
    ImGui::Separator();
    ImGui::Spacing();

    // Dataset path
    ImGui::Text("Dataset path (CSV)");
    ImGui::SetNextItemWidth(-1);
    ImGui::InputText("##path", dataset_path_buf_, sizeof(dataset_path_buf_));
    ImGui::Spacing();

    // Options row
    ImGui::Text("Estimator backend");
    ImGui::SameLine(160);
    ImGui::SetNextItemWidth(120);
    const char* backends[] = {"EKF", "UKF", "Ceres", "GTSAM"};
    ImGui::Combo("##bk", &backend_idx_, backends, IM_ARRAYSIZE(backends));

    ImGui::SameLine(300);
    ImGui::Text("Playback speed");
    ImGui::SameLine(420);
    ImGui::SetNextItemWidth(100);
    ImGui::SliderFloat("##spd", &playback_speed_, 0.1f, 10.0f, "%.1f x");

    ImGui::Spacing();

    // Advanced JSON toggle
    if (ImGui::CollapsingHeader("Advanced — override JSON config")) {
        if (!config_json_dirty_) {
            app::SessionConfig defaults;
            auto j = nlohmann::json::parse(defaults.ToJson());
            std::snprintf(config_json_buf_, sizeof(config_json_buf_), "%s",
                          j.dump(2).c_str());
        }
        ImGui::InputTextMultiline("##json", config_json_buf_, sizeof(config_json_buf_),
                                  ImVec2(-1, 120));
        config_json_dirty_ = true;
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Start button — disabled if no path
    bool can_start = dataset_path_buf_[0] != '\0';
    if (!can_start) {
        ImGui::BeginDisabled();
        ImGui::Button("Start Session", {-1, 36});
        ImGui::EndDisabled();
        ImGui::TextDisabled("  Select a dataset path first");
    } else {
        if (ImGui::Button("Start Session", {-1, 36}))
            StartSession();
    }

    ImGui::End();
}

// ── Main screen ───────────────────────────────────────────────────────────────

void UiApp::DrawMainScreen() {
    int fw, fh;
    glfwGetFramebufferSize(window_, &fw, &fh);

    const float status_h  = ImGui::GetFrameHeight() + 4.0f;
    const float usable_h  = float(fh) - status_h;
    const float left_w    = float(fw) * 0.55f;
    const float right_w   = float(fw) - left_w;
    const float top_h     = usable_h * 0.65f;
    const float bottom_h  = usable_h - top_h;

    // Trajectory (top-left)
    ImGui::SetNextWindowPos({0, 0});
    ImGui::SetNextWindowSize({left_w, top_h});
    DrawTrajectoryPanel();

    // Charts (top-right)
    ImGui::SetNextWindowPos({left_w, 0});
    ImGui::SetNextWindowSize({right_w, top_h});
    DrawChartPanel();

    // Controls (bottom-left)
    ImGui::SetNextWindowPos({0, top_h});
    ImGui::SetNextWindowSize({left_w, bottom_h});
    DrawControlPanel();

    // Status bar (bottom)
    DrawStatusBar();
}

// ── Status bar ────────────────────────────────────────────────────────────────

void UiApp::DrawStatusBar() {
    int fw, fh;
    glfwGetFramebufferSize(window_, &fw, &fh);
    const float bar_h = ImGui::GetFrameHeight() + 4.0f;
    ImGui::SetNextWindowPos({0.0f, float(fh) - bar_h});
    ImGui::SetNextWindowSize({float(fw), bar_h});
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::Begin("##statusbar", nullptr,
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoInputs   | ImGuiWindowFlags_NoNav |
        ImGuiWindowFlags_NoMove     | ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoBringToFrontOnFocus);
    ImGui::PopStyleVar();

    std::string status;
    uint64_t mread;
    bool connected = ws_client_.IsConnected();
    {
        std::lock_guard lk(mu_);
        status = status_;
        mread  = meas_read_;
    }

    ImVec4 conn_col = connected ? ImVec4(0.2f,0.8f,0.3f,1.0f) : ImVec4(0.8f,0.4f,0.2f,1.0f);
    ImGui::TextColored(conn_col, connected ? "● Connected" : "○ Connecting...");
    ImGui::SameLine(0, 20);
    ImGui::Text("Status: %s", status.c_str());
    ImGui::SameLine(0, 20);
    ImGui::Text("Measurements: %llu", static_cast<unsigned long long>(mread));
    ImGui::SameLine(0, 20);
    ImGui::TextDisabled("ws://127.0.0.1:%u", unsigned(ws_port_));
    ImGui::End();
}

// ── Trajectory ────────────────────────────────────────────────────────────────

void UiApp::DrawTrajectoryPanel() {
    ImGui::Begin("Trajectory (ENU)", nullptr,
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

    std::vector<double> east, north;
    double lat0 = 0.0, lon0 = 0.0;
    {
        std::lock_guard lk(mu_);
        if (!samples_.empty()) {
            lat0 = samples_.front().lat_deg;
            lon0 = samples_.front().lon_deg;
            constexpr double kDeg2Rad = M_PI / 180.0;
            constexpr double kR       = 6378137.0;
            for (auto& s : samples_) {
                double dlat = (s.lat_deg - lat0) * kDeg2Rad * kR;
                double dlon = (s.lon_deg - lon0) * kDeg2Rad * kR *
                              std::cos(lat0 * kDeg2Rad);
                east.push_back(dlon);
                north.push_back(dlat);
            }
        }
    }

    ImGui::Text("Origin: %.6f°N  %.6f°E", lat0, lon0);
    if (ImPlot::BeginPlot("##enu", ImVec2(-1, -1),
                          ImPlotFlags_Equal | ImPlotFlags_NoTitle)) {
        ImPlot::SetupAxes("East (m)", "North (m)");
        if (!east.empty()) {
            ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 2.0f);
            ImPlot::PlotScatter("Path", east.data(), north.data(),
                                static_cast<int>(east.size()));
            ImPlot::SetNextMarkerStyle(ImPlotMarker_Diamond, 10.0f,
                                       ImVec4(1.0f, 0.3f, 0.3f, 1.0f));
            ImPlot::PlotScatter("Now", &east.back(), &north.back(), 1);
        }
        ImPlot::EndPlot();
    }
    ImGui::End();
}

// ── Charts ────────────────────────────────────────────────────────────────────

void UiApp::DrawChartPanel() {
    ImGui::Begin("Charts", nullptr,
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

    std::vector<double> ts, ve, vn, vu, roll, pitch, yaw, ph, pv;
    {
        std::lock_guard lk(mu_);
        ts    = col(samples_, [](auto& s){ return s.t_s; });
        ve    = col(samples_, [](auto& s){ return s.vel_e; });
        vn    = col(samples_, [](auto& s){ return s.vel_n; });
        vu    = col(samples_, [](auto& s){ return s.vel_u; });
        roll  = col(samples_, [](auto& s){ return s.roll_deg; });
        pitch = col(samples_, [](auto& s){ return s.pitch_deg; });
        yaw   = col(samples_, [](auto& s){ return s.yaw_deg; });
        ph    = col(samples_, [](auto& s){ return s.pos_std_h; });
        pv    = col(samples_, [](auto& s){ return s.pos_std_v; });
    }
    int n = static_cast<int>(ts.size());

    if (ImGui::BeginTabBar("##tabs")) {
        if (ImGui::BeginTabItem("Velocity")) {
            if (ImPlot::BeginPlot("##vel", ImVec2(-1, -1))) {
                ImPlot::SetupAxes("Time (s)", "m/s");
                if (n > 0) {
                    ImPlot::PlotLine("East",  ts.data(), ve.data(), n);
                    ImPlot::PlotLine("North", ts.data(), vn.data(), n);
                    ImPlot::PlotLine("Up",    ts.data(), vu.data(), n);
                }
                ImPlot::EndPlot();
            }
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Attitude")) {
            if (ImPlot::BeginPlot("##att", ImVec2(-1, -1))) {
                ImPlot::SetupAxes("Time (s)", "deg");
                if (n > 0) {
                    ImPlot::PlotLine("Roll",  ts.data(), roll.data(),  n);
                    ImPlot::PlotLine("Pitch", ts.data(), pitch.data(), n);
                    ImPlot::PlotLine("Yaw",   ts.data(), yaw.data(),   n);
                }
                ImPlot::EndPlot();
            }
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Pos Std")) {
            if (ImPlot::BeginPlot("##pstd", ImVec2(-1, -1))) {
                ImPlot::SetupAxes("Time (s)", "m");
                if (n > 0) {
                    ImPlot::PlotLine("Horizontal", ts.data(), ph.data(), n);
                    ImPlot::PlotLine("Vertical",   ts.data(), pv.data(), n);
                }
                ImPlot::EndPlot();
            }
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    ImGui::End();
}

// ── Controls ──────────────────────────────────────────────────────────────────

void UiApp::DrawControlPanel() {
    ImGui::Begin("Controls & State", nullptr,
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

    // Replay controls
    if (ImGui::Button("Pause",  {80,0})) SendCmd({{"cmd","pause"}});
    ImGui::SameLine();
    if (ImGui::Button("Resume", {80,0})) SendCmd({{"cmd","resume"}});
    ImGui::SameLine();
    if (ImGui::Button("Stop & back", {100,0})) StopSession();

    ImGui::SameLine(0, 20);
    ImGui::Text("Speed");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(120);
    if (ImGui::SliderFloat("##spd2", &playback_speed_, 0.1f, 10.0f, "%.1f x"))
        SendCmd({{"cmd","set_speed"},{"speed", double(playback_speed_)}});

    ImGui::Separator();

    // Nav state values
    NavSample cur{};
    {
        std::lock_guard lk(mu_);
        if (!samples_.empty()) cur = samples_.back();
    }

    ImGui::Columns(2, "##nav", false);
    ImGui::Text("Lat");       ImGui::NextColumn(); ImGui::Text("%.7f °", cur.lat_deg);  ImGui::NextColumn();
    ImGui::Text("Lon");       ImGui::NextColumn(); ImGui::Text("%.7f °", cur.lon_deg);  ImGui::NextColumn();
    ImGui::Text("Alt");       ImGui::NextColumn(); ImGui::Text("%.2f m",  cur.alt_m);   ImGui::NextColumn();
    ImGui::Text("Roll");      ImGui::NextColumn(); ImGui::Text("%.2f °", cur.roll_deg); ImGui::NextColumn();
    ImGui::Text("Pitch");     ImGui::NextColumn(); ImGui::Text("%.2f °", cur.pitch_deg);ImGui::NextColumn();
    ImGui::Text("Yaw");       ImGui::NextColumn(); ImGui::Text("%.2f °", cur.yaw_deg);  ImGui::NextColumn();
    ImGui::Text("Vel E/N/U"); ImGui::NextColumn(); ImGui::Text("%.2f / %.2f / %.2f m/s", cur.vel_e, cur.vel_n, cur.vel_u); ImGui::NextColumn();
    ImGui::Text("PosStd H/V");ImGui::NextColumn(); ImGui::Text("%.3f / %.3f m", cur.pos_std_h, cur.pos_std_v); ImGui::NextColumn();
    ImGui::Columns(1);

    ImGui::End();
}

}  // namespace falconguide::ui
