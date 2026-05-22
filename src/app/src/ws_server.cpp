#include "falconguide/app/ws_server.hpp"
#include "falconguide/logger/logger.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace falconguide::app {

// ── SHA-1 (RFC 3174) — needed for the WebSocket handshake accept key ─────────

void WsServer::Sha1(const uint8_t* data, std::size_t len, uint8_t out[20]) {
    auto R = [](uint32_t v, int s) -> uint32_t { return (v << s) | (v >> (32 - s)); };
    uint32_t h[5] = {0x67452301u, 0xEFCDAB89u, 0x98BADCFEu, 0x10325476u, 0xC3D2E1F0u};

    std::vector<uint8_t> m(data, data + len);
    m.push_back(0x80);
    while (m.size() % 64 != 56) m.push_back(0x00);
    const uint64_t bl = uint64_t(len) * 8;
    for (int i = 7; i >= 0; --i) m.push_back(uint8_t(bl >> (i * 8)));

    for (std::size_t i = 0; i < m.size(); i += 64) {
        uint32_t w[80];
        for (int j = 0; j < 16; ++j)
            w[j] = (uint32_t(m[i + j * 4]) << 24) | (uint32_t(m[i + j * 4 + 1]) << 16) |
                   (uint32_t(m[i + j * 4 + 2]) << 8) | uint32_t(m[i + j * 4 + 3]);
        for (int j = 16; j < 80; ++j) w[j] = R(w[j - 3] ^ w[j - 8] ^ w[j - 14] ^ w[j - 16], 1);

        uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];
        for (int j = 0; j < 80; ++j) {
            uint32_t f, k;
            if (j < 20) {
                f = (b & c) | (~b & d);
                k = 0x5A827999u;
            } else if (j < 40) {
                f = b ^ c ^ d;
                k = 0x6ED9EBA1u;
            } else if (j < 60) {
                f = (b & c) | (b & d) | (c & d);
                k = 0x8F1BBCDCu;
            } else {
                f = b ^ c ^ d;
                k = 0xCA62C1D6u;
            }
            uint32_t t = R(a, 5) + f + e + k + w[j];
            e = d;
            d = c;
            c = R(b, 30);
            b = a;
            a = t;
        }
        h[0] += a;
        h[1] += b;
        h[2] += c;
        h[3] += d;
        h[4] += e;
    }
    for (int i = 0; i < 5; ++i) {
        out[i * 4]     = uint8_t(h[i] >> 24);
        out[i * 4 + 1] = uint8_t(h[i] >> 16);
        out[i * 4 + 2] = uint8_t(h[i] >> 8);
        out[i * 4 + 3] = uint8_t(h[i]);
    }
}

// ── Base64 ────────────────────────────────────────────────────────────────────

std::string WsServer::Base64(const uint8_t* d, std::size_t n) {
    static constexpr const char* T =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string o;
    o.reserve(((n + 2) / 3) * 4);
    for (std::size_t i = 0; i < n; i += 3) {
        uint32_t v = uint32_t(d[i]) << 16;
        if (i + 1 < n) v |= uint32_t(d[i + 1]) << 8;
        if (i + 2 < n) v |= uint32_t(d[i + 2]);
        o += T[(v >> 18) & 63];
        o += T[(v >> 12) & 63];
        o += (i + 1 < n) ? T[(v >> 6) & 63] : '=';
        o += (i + 2 < n) ? T[v & 63] : '=';
    }
    return o;
}

// ── Constructor / Destructor ──────────────────────────────────────────────────

WsServer::WsServer(WsConfig cfg, CommandHandler handler)
    : cfg_(std::move(cfg)), handler_(std::move(handler)) {}

WsServer::~WsServer() { Stop(); }

// ── Start / Stop ──────────────────────────────────────────────────────────────

void WsServer::Start() {
    if (running_.load()) return;

    server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) throw std::runtime_error("WsServer: socket() failed");

    int opt = 1;
    ::setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    ::fcntl(server_fd_, F_SETFL, O_NONBLOCK);

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(cfg_.port);

    if (::bind(server_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(server_fd_);
        server_fd_ = -1;
        throw std::runtime_error("WsServer: bind() failed on port " + std::to_string(cfg_.port));
    }
    ::listen(server_fd_, 16);

    running_.store(true);
    thread_ = std::thread(&WsServer::Loop, this);
    FG_INFO("WsServer | listening on port {}", cfg_.port);
}

void WsServer::Stop() {
    if (!running_.exchange(false)) return;
    if (server_fd_ >= 0) {
        ::close(server_fd_);
        server_fd_ = -1;
    }
    if (thread_.joinable()) thread_.join();
    std::lock_guard lk(mu_);
    for (auto& c : clients_)
        if (c.fd >= 0) ::close(c.fd);
    clients_.clear();
}

bool WsServer::IsRunning() const { return running_.load(); }

// ── Server Loop ───────────────────────────────────────────────────────────────

void WsServer::Loop() {
    while (running_.load()) {
        fd_set rfds;
        FD_ZERO(&rfds);
        int maxfd = server_fd_;
        FD_SET(server_fd_, &rfds);

        {
            std::lock_guard lk(mu_);
            for (const auto& c : clients_) {
                if (!c.dead && c.fd >= 0) {
                    FD_SET(c.fd, &rfds);
                    if (c.fd > maxfd) maxfd = c.fd;
                }
            }
        }

        timeval tv{0, 50'000};
        if (::select(maxfd + 1, &rfds, nullptr, nullptr, &tv) <= 0) continue;

        // Accept new connections
        if (FD_ISSET(server_fd_, &rfds)) {
            const int cfd = ::accept(server_fd_, nullptr, nullptr);
            if (cfd >= 0) {
                std::lock_guard lk(mu_);
                if (static_cast<int>(clients_.size()) < cfg_.max_clients) {
                    ::fcntl(cfd, F_SETFL, O_NONBLOCK);
                    clients_.push_back({cfd, {}, false, false});
                } else {
                    ::close(cfd);
                }
            }
        }

        // Read from existing clients
        {
            std::lock_guard lk(mu_);
            for (auto& c : clients_) {
                if (c.dead || c.fd < 0 || !FD_ISSET(c.fd, &rfds)) continue;
                if (!HandleRead(c)) {
                    ::close(c.fd);
                    c.fd   = -1;
                    c.dead = true;
                }
            }
            clients_.erase(
                std::remove_if(clients_.begin(), clients_.end(), [](const Client& c) { return c.dead; }),
                clients_.end());
        }
    }
}

// ── HandleRead ────────────────────────────────────────────────────────────────

bool WsServer::HandleRead(Client& c) {
    char buf[4096];
    const ssize_t n = ::read(c.fd, buf, sizeof(buf));
    if (n <= 0) return false;
    c.buf.append(buf, std::size_t(n));

    if (!c.upgraded) {
        if (c.buf.find("\r\n\r\n") == std::string::npos) return true;  // need more headers

        if (c.buf.find("Upgrade: websocket") != std::string::npos ||
            c.buf.find("Upgrade: WebSocket") != std::string::npos) {
            return DoUpgrade(c);
        }
        ServeHttp(c);
        return false;
    }

    // Drain WebSocket frames
    std::string payload;
    while (RecvFrame(c, payload)) {
        try {
            const auto resp = handler_(nlohmann::json::parse(payload));
            if (!resp.is_null()) SendText(c.fd, resp.dump());
        } catch (...) {
            SendText(c.fd, R"({"event":"error","message":"bad JSON"})");
        }
        payload.clear();
    }
    return true;
}

// ── DoUpgrade ─────────────────────────────────────────────────────────────────

bool WsServer::DoUpgrade(Client& c) {
    const std::string hdr  = "Sec-WebSocket-Key: ";
    const auto        pos  = c.buf.find(hdr);
    if (pos == std::string::npos) return false;
    const auto end = c.buf.find("\r\n", pos + hdr.size());
    if (end == std::string::npos) return false;
    const std::string key    = c.buf.substr(pos + hdr.size(), end - pos - hdr.size());
    const std::string magic  = key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    uint8_t           sha[20];
    Sha1(reinterpret_cast<const uint8_t*>(magic.data()), magic.size(), sha);
    const std::string accept = Base64(sha, 20);

    const std::string resp =
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Accept: " +
        accept + "\r\n\r\n";
    [[maybe_unused]] auto _ = ::write(c.fd, resp.data(), resp.size());
    c.upgraded = true;
    c.buf.clear();
    return true;
}

// ── ServeHttp ─────────────────────────────────────────────────────────────────

void WsServer::ServeHttp(Client& c) {
    std::string body;
    std::ifstream f(cfg_.ui_path);
    if (f) {
        std::ostringstream ss;
        ss << f.rdbuf();
        body = ss.str();
    } else {
        body =
            "<!DOCTYPE html><html><body>"
            "<h2>FalconGuide UI</h2>"
            "<p>UI file not found at <code>" +
            cfg_.ui_path +
            "</code>. "
            "Build with <code>cmake --build</code> or set <code>ws.ui_path</code>.</p>"
            "</body></html>";
    }
    const std::string resp =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html; charset=utf-8\r\n"
        "Content-Length: " +
        std::to_string(body.size()) +
        "\r\n"
        "Connection: close\r\n\r\n" +
        body;
    [[maybe_unused]] auto _ = ::write(c.fd, resp.data(), resp.size());
}

// ── WebSocket Frame I/O ───────────────────────────────────────────────────────

bool WsServer::RecvFrame(Client& c, std::string& payload) {
    auto& buf = c.buf;
    if (buf.size() < 2) return false;

    const uint8_t b0     = uint8_t(buf[0]);
    const uint8_t b1     = uint8_t(buf[1]);
    const uint8_t opcode = b0 & 0x0Fu;
    const bool    masked  = (b1 & 0x80u) != 0;
    uint64_t      plen    = b1 & 0x7Fu;

    std::size_t ext = (plen == 126) ? 2u : (plen == 127) ? 8u : 0u;
    std::size_t hdr = 2u + ext + (masked ? 4u : 0u);

    if (buf.size() < 2 + ext) return false;

    if (plen == 126) {
        plen = (uint64_t(uint8_t(buf[2])) << 8) | uint64_t(uint8_t(buf[3]));
    } else if (plen == 127) {
        plen = 0;
        for (int i = 0; i < 8; ++i) plen = (plen << 8) | uint64_t(uint8_t(buf[2 + i]));
    }

    if (buf.size() < hdr + plen) return false;

    if (opcode == 0x8u) {  // close
        SendClose(c.fd);
        buf.erase(0, hdr + std::size_t(plen));
        c.dead = true;
        return false;
    }

    const std::size_t mask_pos = 2 + ext;
    const std::size_t data_pos = mask_pos + (masked ? 4u : 0u);

    payload.resize(std::size_t(plen));
    for (uint64_t i = 0; i < plen; ++i) {
        auto byte = uint8_t(buf[data_pos + i]);
        if (masked) byte ^= uint8_t(buf[mask_pos + (i % 4)]);
        payload[i] = char(byte);
    }
    buf.erase(0, hdr + std::size_t(plen));
    return opcode == 0x1u;  // text frame
}

void WsServer::SendText(int fd, const std::string& text) {
    if (text.empty()) return;
    const std::size_t plen = text.size();
    std::string       frame;
    frame.reserve(plen + 10);
    frame.push_back(char(0x81u));  // FIN + text opcode

    if (plen < 126) {
        frame.push_back(char(plen));
    } else if (plen < 65536) {
        frame.push_back(char(126));
        frame.push_back(char(plen >> 8));
        frame.push_back(char(plen & 0xFFu));
    } else {
        frame.push_back(char(127));
        for (int i = 7; i >= 0; --i) frame.push_back(char(plen >> (i * 8)));
    }
    frame += text;
    [[maybe_unused]] auto _ = ::write(fd, frame.data(), frame.size());
}

void WsServer::SendClose(int fd) {
    const char f[] = {char(0x88u), char(0x00u)};
    [[maybe_unused]] auto _ = ::write(fd, f, sizeof(f));
}

// ── Broadcast ─────────────────────────────────────────────────────────────────

void WsServer::Broadcast(const nlohmann::json& msg) {
    const std::string text = msg.dump();
    std::lock_guard   lk(mu_);
    for (auto& c : clients_) {
        if (!c.dead && c.upgraded && c.fd >= 0) SendText(c.fd, text);
    }
}

}  // namespace falconguide::app
