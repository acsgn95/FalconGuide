#include "falconguide/ui/tile_map.hpp"

#include <GL/gl.h>
#include <implot.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>

namespace falconguide::ui {

namespace fs = std::filesystem;

// ── Tile coordinate math (OSM XYZ scheme) ────────────────────────────────────

TileMap::TileKey TileMap::LatLonToTile(double lat_deg, double lon_deg, int zoom) {
    const int n = 1 << zoom;
    const double lat_r = lat_deg * M_PI / 180.0;
    int x = static_cast<int>((lon_deg + 180.0) / 360.0 * n);
    int y = static_cast<int>((1.0 - std::log(std::tan(lat_r) + 1.0 / std::cos(lat_r)) / M_PI) / 2.0 * n);
    x = std::clamp(x, 0, n - 1);
    y = std::clamp(y, 0, n - 1);
    return {x, y, zoom};
}

// Returns (lat_deg, lon_deg) of the top-left corner of tile (tx,ty) at zoom.
std::pair<double, double> TileMap::TileTopLeft(int tx, int ty, int zoom) {
    const int n = 1 << zoom;
    const double lon = tx / double(n) * 360.0 - 180.0;
    const double lat_r = std::atan(std::sinh(M_PI * (1.0 - 2.0 * ty / double(n))));
    return {lat_r * 180.0 / M_PI, lon};
}

// ── Tile file paths / URLs ────────────────────────────────────────────────────

std::string TileMap::CachePath(const TileKey& k) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "/tmp/fg_tiles/%d/%d/%d.png", k.z, k.x, k.y);
    return buf;
}

std::string TileMap::TileUrl(const TileKey& k) {
    // Rotate among a/b/c subdomains to avoid rate-limiting
    const char sub = "abc"[(k.x + k.y) % 3];
    char buf[256];
    std::snprintf(buf, sizeof(buf), "https://%c.tile.openstreetmap.org/%d/%d/%d.png", sub, k.z, k.x, k.y);
    return buf;
}

// ── Constructor / Destructor ──────────────────────────────────────────────────

TileMap::TileMap() {
    fs::create_directories("/tmp/fg_tiles");
    worker_ = std::thread(&TileMap::WorkerLoop, this);
}

TileMap::~TileMap() { Shutdown(); }

void TileMap::Shutdown() {
    stop_ = true;
    worker_.join();
}

// ── Worker ────────────────────────────────────────────────────────────────────

void TileMap::WorkerLoop() {
    while (!stop_) {
        TileKey key{};
        bool has_work = false;
        {
            std::lock_guard lk(mu_);
            if (!work_queue_.empty()) {
                key = work_queue_.front();
                work_queue_.pop();
                has_work = true;
            }
        }
        if (!has_work) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }
        DownloadTile(key);
    }
}

void TileMap::DownloadTile(const TileKey& k) {
    const std::string path = CachePath(k);

    // Try loading from disk cache first
    if (!fs::exists(path)) {
        // Ensure directory exists
        fs::create_directories(fs::path(path).parent_path());

        // Download with curl (timeout 10s, silent)
        const std::string url = TileUrl(k);
        char cmd[512];
        std::snprintf(cmd, sizeof(cmd), "curl -s --max-time 10 -A 'FalconGuide/1.0' -o '%s' '%s' 2>/dev/null",
                      path.c_str(), url.c_str());
        if (std::system(cmd) != 0 || !fs::exists(path)) {
            std::lock_guard lk(mu_);
            if (auto it = tiles_.find(k); it != tiles_.end()) it->second.state = TileState::Failed;
            return;
        }
    }

    cv::Mat img = cv::imread(path, cv::IMREAD_COLOR);
    if (img.empty()) {
        std::lock_guard lk(mu_);
        if (auto it = tiles_.find(k); it != tiles_.end()) it->second.state = TileState::Failed;
        return;
    }

    cv::Mat rgba;
    cv::cvtColor(img, rgba, cv::COLOR_BGR2RGBA);

    std::lock_guard lk(mu_);
    auto it = tiles_.find(k);
    if (it == tiles_.end()) return;
    auto& entry = it->second;
    entry.pixels.assign(rgba.data, rgba.data + rgba.total() * rgba.elemSize());
    entry.w = rgba.cols;
    entry.h = rgba.rows;
    entry.state = TileState::Ready;
}

// ── Update (called each frame on render thread) ───────────────────────────────

void TileMap::Update(double lat0_deg, double lon0_deg, double lat_min, double lat_max, double lon_min, double lon_max) {
    (void)lon0_deg;  // used via lat0_deg for zoom selection
    (void)lat0_deg;

    // Auto-select zoom based on trajectory extent
    const double dlat = std::abs(lat_max - lat_min);
    const double dlon = std::abs(lon_max - lon_min);
    const double extent_deg = std::max(dlat, dlon);
    if (extent_deg < 0.002)
        zoom_ = 18;
    else if (extent_deg < 0.005)
        zoom_ = 17;
    else if (extent_deg < 0.01)
        zoom_ = 16;
    else if (extent_deg < 0.02)
        zoom_ = 15;
    else if (extent_deg < 0.05)
        zoom_ = 14;
    else
        zoom_ = 13;

    // Expand bounds by one tile margin
    const double pad = 360.0 / double(1 << zoom_);

    const auto tl = LatLonToTile(lat_max + pad, lon_min - pad, zoom_);
    const auto br = LatLonToTile(lat_min - pad, lon_max + pad, zoom_);

    std::lock_guard lk(mu_);
    for (int tx = tl.x; tx <= br.x; ++tx) {
        for (int ty = tl.y; ty <= br.y; ++ty) {
            TileKey key{tx, ty, zoom_};
            if (tiles_.count(key)) continue;
            tiles_[key].state = TileState::Loading;
            work_queue_.push(key);
        }
    }
}

// ── RenderInPlot (called inside BeginPlot/EndPlot) ────────────────────────────

void TileMap::RenderInPlot(double lat0_deg, double lon0_deg) {
    ImDrawList* dl = ImPlot::GetPlotDrawList();
    if (!dl) return;

    constexpr double kDeg2Rad = M_PI / 180.0;
    constexpr double kR = 6378137.0;
    const double cos_lat0 = std::cos(lat0_deg * kDeg2Rad);

    // Helper: (lat_deg, lon_deg) → ENU meters (east, north) relative to lat0/lon0
    auto ll_to_enu = [&](double lat, double lon) -> ImPlotPoint {
        double north = (lat - lat0_deg) * kDeg2Rad * kR;
        double east = (lon - lon0_deg) * kDeg2Rad * kR * cos_lat0;
        return {east, north};
    };

    // Upload any ready tiles and draw them
    std::lock_guard lk(mu_);
    for (auto& [key, entry] : tiles_) {
        if (key.z != zoom_) continue;

        // Upload pixels if ready and not yet on GPU
        if (entry.state == TileState::Ready && entry.tex == 0 && !entry.pixels.empty()) {
            GLuint tex = 0;
            glGenTextures(1, &tex);
            glBindTexture(GL_TEXTURE_2D, tex);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, entry.w, entry.h, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                         entry.pixels.data());
            glBindTexture(GL_TEXTURE_2D, 0);
            entry.tex = tex;
            entry.pixels.clear();
            entry.pixels.shrink_to_fit();
        }

        if (entry.tex == 0) continue;

        // Tile corners in lat/lon
        auto [lat_tl, lon_tl] = TileTopLeft(key.x, key.y, key.z);
        auto [lat_br, lon_br] = TileTopLeft(key.x + 1, key.y + 1, key.z);

        // Convert to ENU, then to screen pixels via ImPlot
        ImPlotPoint p_tl = ll_to_enu(lat_tl, lon_tl);
        ImPlotPoint p_br = ll_to_enu(lat_br, lon_br);

        ImVec2 s_tl = ImPlot::PlotToPixels(p_tl);
        ImVec2 s_br = ImPlot::PlotToPixels(p_br);

        dl->AddImage(reinterpret_cast<ImTextureID>(static_cast<uintptr_t>(entry.tex)), s_tl, s_br, {0, 0}, {1, 1},
                     IM_COL32(255, 255, 255, 210));
    }
}

}  // namespace falconguide::ui
