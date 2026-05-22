#include "falconguide/app/ipc_server.hpp"
#include "falconguide/core/coordinates.hpp"

#include <cstring>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <cmath>
#include <stdexcept>
#include <unordered_map>

namespace falconguide::app {

using json = nlohmann::json;

// ── Constructor / Destructor ──────────────────────────────────────────────────

IpcServer::IpcServer(IpcConfig cfg, CommandHandler handler) : cfg_(std::move(cfg)), handler_(std::move(handler)) {}

IpcServer::~IpcServer() { Stop(); }

// ── Start / Stop ──────────────────────────────────────────────────────────────

bool IpcServer::Start() {
    if (running_.load()) return true;

    // Remove stale socket file
    ::unlink(cfg_.socket_path.c_str());

    listen_fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (listen_fd_ < 0) return false;

    // Non-blocking listen socket
    ::fcntl(listen_fd_, F_SETFL, O_NONBLOCK);

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, cfg_.socket_path.c_str(), sizeof(addr.sun_path) - 1);

    if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(listen_fd_);
        listen_fd_ = -1;
        return false;
    }

    if (::listen(listen_fd_, cfg_.max_clients) < 0) {
        ::close(listen_fd_);
        listen_fd_ = -1;
        return false;
    }

    running_.store(true);
    thread_ = std::thread(&IpcServer::ServerLoop, this);
    return true;
}

void IpcServer::Stop() {
    if (!running_.exchange(false)) return;

    if (listen_fd_ >= 0) {
        ::shutdown(listen_fd_, SHUT_RDWR);
        ::close(listen_fd_);
        listen_fd_ = -1;
    }

    {
        std::lock_guard lk(clients_mutex_);
        for (int fd : clients_) ::close(fd);
        clients_.clear();
    }

    if (thread_.joinable()) thread_.join();

    ::unlink(cfg_.socket_path.c_str());
}

// ── ServerLoop ────────────────────────────────────────────────────────────────

void IpcServer::ServerLoop() {
    // Per-client incomplete line buffer
    std::unordered_map<int, std::string> buffers;

    while (running_.load()) {
        fd_set rfds;
        FD_ZERO(&rfds);

        int maxfd = listen_fd_;
        FD_SET(listen_fd_, &rfds);

        {
            std::lock_guard lk(clients_mutex_);
            for (int fd : clients_) {
                FD_SET(fd, &rfds);
                if (fd > maxfd) maxfd = fd;
            }
        }

        timeval tv{0, 50'000};  // 50 ms timeout so we can check running_
        const int ready = ::select(maxfd + 1, &rfds, nullptr, nullptr, &tv);
        if (ready <= 0) continue;

        // Accept new connection
        if (FD_ISSET(listen_fd_, &rfds)) {
            const int cfd = ::accept(listen_fd_, nullptr, nullptr);
            if (cfd >= 0) {
                ::fcntl(cfd, F_SETFL, O_NONBLOCK);
                std::lock_guard lk(clients_mutex_);
                if (static_cast<int>(clients_.size()) < cfg_.max_clients) {
                    clients_.push_back(cfd);
                    buffers[cfd];
                } else {
                    ::close(cfd);
                }
            }
        }

        // Read from clients
        std::vector<int> to_remove;
        {
            std::lock_guard lk(clients_mutex_);
            for (int fd : clients_) {
                if (!FD_ISSET(fd, &rfds)) continue;

                char buf[4096];
                const ssize_t n = ::read(fd, buf, sizeof(buf));
                if (n <= 0) {
                    to_remove.push_back(fd);
                    continue;
                }

                auto& line_buf = buffers[fd];
                line_buf.append(buf, static_cast<std::size_t>(n));

                std::size_t pos;
                while ((pos = line_buf.find('\n')) != std::string::npos) {
                    HandleData(fd, line_buf.substr(0, pos));
                    line_buf.erase(0, pos + 1);
                }
            }
        }

        for (int fd : to_remove) {
            buffers.erase(fd);
            RemoveClient(fd);
        }
    }
}

// ── HandleData ────────────────────────────────────────────────────────────────

void IpcServer::HandleData(int fd, const std::string& line) {
    if (line.empty()) return;
    try {
        const auto cmd = json::parse(line);
        if (handler_) handler_(cmd, fd);
    } catch (...) {
        WriteToClient(fd, json{{"event", "error"}, {"message", "invalid JSON"}}.dump() + "\n");
    }
}

// ── Helpers ───────────────────────────────────────────────────────────────────

void IpcServer::RemoveClient(int fd) {
    std::lock_guard lk(clients_mutex_);
    auto it = std::find(clients_.begin(), clients_.end(), fd);
    if (it != clients_.end()) clients_.erase(it);
    ::close(fd);
}

void IpcServer::WriteToClient(int fd, const std::string& msg) {
    [[maybe_unused]] auto _ = ::write(fd, msg.c_str(), msg.size());
}

// ── Broadcast ─────────────────────────────────────────────────────────────────

void IpcServer::Broadcast(const json& event) {
    const std::string msg = event.dump() + "\n";
    std::lock_guard lk(clients_mutex_);
    std::vector<int> dead;
    for (int fd : clients_) {
        if (::write(fd, msg.c_str(), msg.size()) < 0) dead.push_back(fd);
    }
    for (int fd : dead) {
        auto it = std::find(clients_.begin(), clients_.end(), fd);
        if (it != clients_.end()) clients_.erase(it);
        ::close(fd);
    }
}

// ── Convenience broadcast builders ────────────────────────────────────────────

nlohmann::json IpcServer::NavStateJson(const core::NavigationState& state) {
    const core::Lla lla = core::EcefToLla(state.position_ecef_m);
    const double lat_deg = lla.latitude_rad * (180.0 / M_PI);
    const double lon_deg = lla.longitude_rad * (180.0 / M_PI);
    const double r2d = 180.0 / M_PI;

    const auto& q = state.orientation_body_to_enu;
    const double roll_deg =
        std::atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y() * q.y())) * r2d;
    const double pitch_deg = std::asin(std::clamp(2 * (q.w() * q.y() - q.z() * q.x()), -1.0, 1.0)) * r2d;
    const double yaw_deg =
        std::atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z())) * r2d;

    const auto cov = state.covariance;
    const double pos_h = std::sqrt(std::max(0.0, (cov(0, 0) + cov(1, 1)) * 0.5));
    const double pos_v = std::sqrt(std::max(0.0, cov(2, 2)));
    const double vel_std = std::sqrt(std::max(0.0, (cov(3, 3) + cov(4, 4) + cov(5, 5)) / 3.0));

    auto nav_status_str = [](core::NavigationStatus s) -> std::string {
        switch (s) {
            case core::NavigationStatus::Nominal:
                return "nominal";
            case core::NavigationStatus::DeadReckoning:
                return "dead_reckoning";
            case core::NavigationStatus::Degraded:
                return "degraded";
            case core::NavigationStatus::Initializing:
                return "initializing";
            case core::NavigationStatus::Fault:
                return "fault";
            default:
                return "unknown";
        }
    };

    int64_t ts_ns = 0;
    if (state.timestamp.has_steady) ts_ns = state.timestamp.steady.nanoseconds_since_epoch().count();

    return {
        {"event", "nav_state"},
        {"timestamp_ns", ts_ns},
        {"lat_deg", lat_deg},
        {"lon_deg", lon_deg},
        {"alt_m", lla.altitude_m},
        {"vel_enu_mps",
         json::array({state.velocity_enu_mps.x(), state.velocity_enu_mps.y(), state.velocity_enu_mps.z()})},
        {"roll_deg", roll_deg},
        {"pitch_deg", pitch_deg},
        {"yaw_deg", yaw_deg},
        {"pos_std_h_m", pos_h},
        {"pos_std_v_m", pos_v},
        {"vel_std_mps", vel_std},
        {"initialized", state.quality.initialized},
        {"nav_status", nav_status_str(state.status)},
    };
}

void IpcServer::BroadcastNavState(const core::NavigationState& state) { Broadcast(NavStateJson(state)); }

void IpcServer::BroadcastStatus(const std::string& status, std::size_t measurements_read) {
    Broadcast({
        {"event", "status"},
        {"status", status},
        {"measurements_read", measurements_read},
    });
}

void IpcServer::BroadcastError(const std::string& message) { Broadcast({{"event", "error"}, {"message", message}}); }

void IpcServer::BroadcastSchema(const json& schema) { Broadcast({{"event", "schema"}, {"schema", schema}}); }

}  // namespace falconguide::app
