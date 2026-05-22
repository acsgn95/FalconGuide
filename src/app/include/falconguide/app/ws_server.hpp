#pragma once

#include <nlohmann/json.hpp>

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace falconguide::app {

struct WsConfig {
    uint16_t port{8765};
    int max_clients{16};
    std::string ui_path{"ui/index.html"};  // served on HTTP GET /
};

// ── WsServer ──────────────────────────────────────────────────────────────────
//
// Minimal RFC 6455 WebSocket server over a plain TCP socket.
//
// On the same port:
//   - GET /      → serves the HTML UI file (WsConfig::ui_path)
//   - WebSocket  → bidirectional JSON messaging
//
// Incoming commands are dispatched to CommandHandler which returns a JSON
// response sent back to the requesting client only.
// Outgoing events are pushed to all connected clients via Broadcast().
//
// No external library dependencies: SHA-1 and Base-64 are implemented inline.
//
class WsServer {
   public:
    using CommandHandler = std::function<nlohmann::json(const nlohmann::json&)>;

    WsServer(WsConfig cfg, CommandHandler handler);
    ~WsServer();

    WsServer(const WsServer&) = delete;
    WsServer& operator=(const WsServer&) = delete;

    void Start();
    void Stop();
    [[nodiscard]] bool IsRunning() const;

    void Broadcast(const nlohmann::json& msg);

   private:
    struct Client {
        int fd{-1};
        std::string buf{};
        bool upgraded{false};
        bool dead{false};
    };

    void Loop();
    bool HandleRead(Client& c);  // returns false → mark dead
    bool DoUpgrade(Client& c);
    void ServeHttp(Client& c);
    bool RecvFrame(Client& c, std::string& payload);  // returns true → got text frame
    void SendText(int fd, const std::string& text);
    void SendClose(int fd);

    static void Sha1(const uint8_t* data, std::size_t len, uint8_t out[20]);
    static std::string Base64(const uint8_t* data, std::size_t len);

    WsConfig cfg_;
    CommandHandler handler_;
    int server_fd_{-1};
    std::atomic<bool> running_{false};
    std::thread thread_;
    std::mutex mu_;
    std::vector<Client> clients_;
};

}  // namespace falconguide::app
