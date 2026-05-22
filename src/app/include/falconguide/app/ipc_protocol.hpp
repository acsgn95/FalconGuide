#pragma once

/**
 * @file ipc_protocol.hpp
 * @brief Newline-delimited JSON IPC protocol reference for FalconGuide
 * sessions.
 *
 * This file is intentionally documentation-only. The protocol uses UTF-8 JSON
 * objects over a Unix domain socket, one message per line.
 */

// ── FalconGuide IPC Protocol (Unix socket, newline-delimited JSON)
// ────────────
//
// All messages are UTF-8 JSON objects terminated by '\n'.
//
// ── Commands (client → server)
// ────────────────────────────────────────────────
//
//   {"cmd": "get_status"}
//   {"cmd": "get_schema"}
//   {"cmd": "start"}
//   {"cmd": "stop"}
//   {"cmd": "pause"}
//   {"cmd": "resume"}
//   {"cmd": "set_speed",   "speed": 2.0}
//   {"cmd": "configure",   "config": { ... full SessionConfig JSON ... }}
//
// ── Events (server → all clients) ────────────────────────────────────────────
//
//   {"event": "status",
//    "status": "running",          //
//    idle|configured|running|paused|completed|error "measurements_read": 4321}
//
//   {"event": "nav_state",
//    "timestamp_ns": 1234567890,
//    "lat_deg": 48.0, "lon_deg": 11.0, "alt_m": 500.0,
//    "pos_enu_m": [e, n, u],
//    "vel_enu_mps": [ve, vn, vu],
//    "roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": 0.0,
//    "pos_std_m": [se, sn, su],
//    "vel_std_mps": [sve, svn, svu],
//    "initialized": true,
//    "nav_status": "nominal"}
//
//   {"event": "error", "message": "..."}
//   {"event": "schema", "schema": { ... ToSchemaJson() ... }}
//
