#pragma once

#include <atomic>
#include <map>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <tuple>

namespace falconguide::ui {

/// Downloads and caches OpenStreetMap tiles; renders them into the active ImPlot.
class TileMap {
   public:
    TileMap();
    ~TileMap();

    /// Call once per frame before BeginPlot — queues any missing tiles for download.
    /// lat0/lon0 are the ENU origin used by DrawTrajectoryPanel.
    void Update(double lat0_deg, double lon0_deg, double lat_min, double lat_max, double lon_min, double lon_max);

    /// Call inside ImPlot::BeginPlot / EndPlot — draws tile textures as plot background.
    /// lat0/lon0 must match what was passed to Update.
    void RenderInPlot(double lat0_deg, double lon0_deg);

    void Shutdown();

   private:
    struct TileKey {
        int x, y, z;
        bool operator<(const TileKey& o) const { return std::tie(x, y, z) < std::tie(o.x, o.y, o.z); }
    };

    enum class TileState { Pending, Loading, Ready, Failed };

    struct TileEntry {
        TileState state{TileState::Pending};
        unsigned int tex{0};
        int w{0}, h{0};
        // raw RGBA bytes ready for GL upload (set by worker, consumed on render thread)
        std::vector<unsigned char> pixels;
    };

    static TileKey LatLonToTile(double lat_deg, double lon_deg, int zoom);
    static std::pair<double, double> TileTopLeft(int tx, int ty, int zoom);
    static std::string CachePath(const TileKey& k);
    static std::string TileUrl(const TileKey& k);

    // Called on render thread to upload pending pixels to GL
    void FlushUploads();
    void WorkerLoop();
    void DownloadTile(const TileKey& k);

    mutable std::mutex mu_;
    std::map<TileKey, TileEntry> tiles_;
    std::queue<TileKey> work_queue_;

    std::thread worker_;
    std::atomic<bool> stop_{false};

    int zoom_{17};
};

}  // namespace falconguide::ui
