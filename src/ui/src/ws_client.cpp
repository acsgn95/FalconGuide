#include "falconguide/ui/ws_client.hpp"

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
using SockLen = int;
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
using SockLen = socklen_t;
#endif

#include <array>
#include <cstdint>
#include <cstring>
#include <random>
#include <stdexcept>

namespace falconguide::ui {

// ── Platform helpers ──────────────────────────────────────────────────────────

static void sock_close(int fd) {
#ifdef _WIN32
    closesocket(static_cast<SOCKET>(fd));
#else
    ::close(fd);
#endif
}

static int sock_connect(const std::string& host, uint16_t port) {
#ifdef _WIN32
    WSADATA wsa{};
    WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    addrinfo* res = nullptr;
    if (getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &res) != 0) return -1;

    int fd = static_cast<int>(::socket(res->ai_family, res->ai_socktype, res->ai_protocol));
    if (fd < 0) {
        freeaddrinfo(res);
        return -1;
    }
    if (::connect(fd, res->ai_addr, static_cast<SockLen>(res->ai_addrlen)) != 0) {
        sock_close(fd);
        freeaddrinfo(res);
        return -1;
    }
    freeaddrinfo(res);
    return fd;
}

static bool sock_send(int fd, const void* data, std::size_t len) {
    const auto* p = static_cast<const char*>(data);
    while (len > 0) {
        int sent = ::send(fd, p, static_cast<int>(len), 0);
        if (sent <= 0) return false;
        p += sent;
        len -= static_cast<std::size_t>(sent);
    }
    return true;
}

static int sock_recv_timeout(int fd, void* buf, int len, int timeout_ms) {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    timeval tv{timeout_ms / 1000, (timeout_ms % 1000) * 1000};
    int r = ::select(fd + 1, &fds, nullptr, nullptr, &tv);
    if (r <= 0) return r;
    return ::recv(fd, static_cast<char*>(buf), len, 0);
}

// ── SHA-1 (RFC 3174) ─────────────────────────────────────────────────────────

void WsClient::Sha1(const uint8_t* data, std::size_t len, uint8_t out[20]) {
    uint32_t h[5] = {0x67452301u, 0xEFCDAB89u, 0x98BADCFEu, 0x10325476u, 0xC3D2E1F0u};
    auto rol = [](uint32_t v, int s) { return (v << s) | (v >> (32 - s)); };

    auto process = [&](const uint8_t blk[64]) {
        uint32_t w[80];
        for (int i = 0; i < 16; i++)
            w[i] = (uint32_t(blk[i * 4]) << 24) | (uint32_t(blk[i * 4 + 1]) << 16) | (uint32_t(blk[i * 4 + 2]) << 8) |
                   blk[i * 4 + 3];
        for (int i = 16; i < 80; i++) w[i] = rol(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
        uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];
        for (int i = 0; i < 80; i++) {
            uint32_t f, k;
            if (i < 20) {
                f = (b & c) | ((~b) & d);
                k = 0x5A827999u;
            } else if (i < 40) {
                f = b ^ c ^ d;
                k = 0x6ED9EBA1u;
            } else if (i < 60) {
                f = (b & c) | (b & d) | (c & d);
                k = 0x8F1BBCDCu;
            } else {
                f = b ^ c ^ d;
                k = 0xCA62C1D6u;
            }
            uint32_t tmp = rol(a, 5) + f + e + k + w[i];
            e = d;
            d = c;
            c = rol(b, 30);
            b = a;
            a = tmp;
        }
        h[0] += a;
        h[1] += b;
        h[2] += c;
        h[3] += d;
        h[4] += e;
    };

    std::vector<uint8_t> msg(data, data + len);
    uint64_t bit_len = uint64_t(len) * 8;
    msg.push_back(0x80);
    while (msg.size() % 64 != 56) msg.push_back(0);
    for (int i = 7; i >= 0; i--) msg.push_back(uint8_t(bit_len >> (i * 8)));
    for (std::size_t i = 0; i < msg.size(); i += 64) process(msg.data() + i);
    for (int i = 0; i < 5; i++) {
        out[i * 4] = uint8_t(h[i] >> 24);
        out[i * 4 + 1] = uint8_t(h[i] >> 16);
        out[i * 4 + 2] = uint8_t(h[i] >> 8);
        out[i * 4 + 3] = uint8_t(h[i]);
    }
}

// ── Base64 ────────────────────────────────────────────────────────────────────

std::string WsClient::Base64(const uint8_t* data, std::size_t len) {
    static const char kAlph[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve(((len + 2) / 3) * 4);
    for (std::size_t i = 0; i < len; i += 3) {
        uint32_t v = uint32_t(data[i]) << 16;
        if (i + 1 < len) v |= uint32_t(data[i + 1]) << 8;
        if (i + 2 < len) v |= uint32_t(data[i + 2]);
        out += kAlph[(v >> 18) & 63];
        out += kAlph[(v >> 12) & 63];
        out += (i + 1 < len) ? kAlph[(v >> 6) & 63] : '=';
        out += (i + 2 < len) ? kAlph[v & 63] : '=';
    }
    return out;
}

// ── WebSocket handshake ───────────────────────────────────────────────────────

bool WsClient::Handshake(int fd) {
    // Random 16-byte key
    std::mt19937 rng(std::random_device{}());
    uint8_t key_bytes[16];
    for (auto& b : key_bytes) b = uint8_t(rng() & 0xFF);
    std::string key = Base64(key_bytes, 16);

    std::string req =
        "GET / HTTP/1.1\r\n"
        "Host: " +
        host_ + ":" + std::to_string(port_) +
        "\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Key: " +
        key +
        "\r\n"
        "Sec-WebSocket-Version: 13\r\n"
        "\r\n";

    if (!sock_send(fd, req.data(), req.size())) return false;

    // Read response until \r\n\r\n
    std::string resp;
    char ch;
    while (resp.size() < 4096) {
        int r = sock_recv_timeout(fd, &ch, 1, 3000);
        if (r <= 0) return false;
        resp += ch;
        if (resp.size() >= 4 && resp.substr(resp.size() - 4) == "\r\n\r\n") break;
    }
    return resp.find("101") != std::string::npos;
}

// ── Frame I/O ─────────────────────────────────────────────────────────────────

bool WsClient::SendFrame(int fd, const std::string& payload) {
    // Client must mask frames (RFC 6455 §5.3)
    std::mt19937 rng(std::random_device{}());
    uint8_t mask[4];
    for (auto& b : mask) b = uint8_t(rng() & 0xFF);

    std::vector<uint8_t> frame;
    frame.push_back(0x81);  // FIN + text opcode
    std::size_t plen = payload.size();
    if (plen <= 125) {
        frame.push_back(uint8_t(plen | 0x80));  // masked
    } else if (plen <= 65535) {
        frame.push_back(0xFE);
        frame.push_back(uint8_t(plen >> 8));
        frame.push_back(uint8_t(plen));
    } else {
        frame.push_back(0xFF);
        for (int i = 7; i >= 0; i--) frame.push_back(uint8_t(plen >> (i * 8)));
    }
    frame.insert(frame.end(), mask, mask + 4);
    for (std::size_t i = 0; i < plen; i++) frame.push_back(uint8_t(payload[i]) ^ mask[i % 4]);

    return sock_send(fd, frame.data(), frame.size());
}

bool WsClient::RecvFrame(int fd, std::string& out) {
    auto recv1 = [&](uint8_t& b) { return sock_recv_timeout(fd, &b, 1, 5000) == 1; };

    uint8_t h0, h1;
    if (!recv1(h0) || !recv1(h1)) return false;

    bool masked = (h1 & 0x80) != 0;
    uint8_t opcode = h0 & 0x0F;
    if (opcode == 8) return false;  // close

    uint64_t plen = h1 & 0x7F;
    if (plen == 126) {
        uint8_t b[2];
        if (!recv1(b[0]) || !recv1(b[1])) return false;
        plen = (uint64_t(b[0]) << 8) | b[1];
    } else if (plen == 127) {
        uint8_t b[8];
        for (auto& x : b)
            if (!recv1(x)) return false;
        plen = 0;
        for (int i = 0; i < 8; i++) plen = (plen << 8) | b[i];
    }

    uint8_t mask[4] = {};
    if (masked)
        for (auto& b : mask)
            if (!recv1(b)) return false;

    out.resize(plen);
    for (uint64_t i = 0; i < plen; i++) {
        uint8_t b;
        if (!recv1(b)) return false;
        out[i] = char(masked ? (b ^ mask[i % 4]) : b);
    }
    return true;
}

// ── Lifecycle ─────────────────────────────────────────────────────────────────

WsClient::WsClient(std::string host, uint16_t port, MessageHandler on_message)
    : host_(std::move(host)), port_(port), on_message_(std::move(on_message)) {}

WsClient::~WsClient() { Disconnect(); }

void WsClient::Connect() {
    if (running_.exchange(true)) return;
    thread_ = std::thread(&WsClient::Loop, this);
}

void WsClient::Disconnect() {
    running_.store(false);
    connected_.store(false);
    if (socket_fd_ >= 0) {
        sock_close(socket_fd_);
        socket_fd_ = -1;
    }
    if (thread_.joinable()) thread_.join();
}

void WsClient::Send(const nlohmann::json& msg) {
    if (!connected_.load() || socket_fd_ < 0) return;
    SendFrame(socket_fd_, msg.dump());
}

void WsClient::Loop() {
    while (running_.load()) {
        int fd = sock_connect(host_, port_);
        if (fd < 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            continue;
        }
        socket_fd_ = fd;

        if (!Handshake(fd)) {
            sock_close(fd);
            socket_fd_ = -1;
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            continue;
        }
        connected_.store(true);

        while (running_.load()) {
            std::string frame;
            if (!RecvFrame(fd, frame)) break;
            try {
                auto j = nlohmann::json::parse(frame);
                if (on_message_) on_message_(j);
            } catch (...) {
            }
        }
        connected_.store(false);
        sock_close(fd);
        socket_fd_ = -1;

        if (running_.load()) std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
}

}  // namespace falconguide::ui
