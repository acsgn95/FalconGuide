#pragma once

/**
 * @file ws_client.hpp
 * @brief Minimal WebSocket client (RFC 6455) — cross-platform (POSIX / Winsock).
 */

#include <nlohmann/json.hpp>

#include <atomic>
#include <functional>
#include <string>
#include <thread>

namespace falconguide::ui {

class WsClient {
   public:
    using MessageHandler = std::function<void(const nlohmann::json&)>;

    explicit WsClient(std::string host, uint16_t port, MessageHandler on_message);
    ~WsClient();

    /// @brief Attempt connection (non-blocking background thread).
    void Connect();
    /// @brief Disconnect and stop background thread.
    void Disconnect();

    bool IsConnected() const { return connected_.load(); }

    /// @brief Send a JSON message (fire-and-forget; ignored when disconnected).
    void Send(const nlohmann::json& msg);

   private:
    void Loop();
    bool Handshake(int fd);
    bool SendFrame(int fd, const std::string& payload);
    bool RecvFrame(int fd, std::string& out);

    static std::string Base64(const uint8_t* data, std::size_t len);
    static void Sha1(const uint8_t* data, std::size_t len, uint8_t out[20]);

    std::string host_;
    uint16_t port_;
    MessageHandler on_message_;

    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    std::thread thread_;

    int socket_fd_{-1};
};

}  // namespace falconguide::ui
