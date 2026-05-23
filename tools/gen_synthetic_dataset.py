#!/usr/bin/env python3
"""
Generate a synthetic FalconGuide CSV dataset.

Simulates a vehicle doing a slow horizontal circle (radius 50 m, ~60 s period)
starting at ETH Zürich. Output is written to the path given as the first argument
(default: /tmp/falconguide_synthetic.csv).

CSV format produced:
  IMU,<ns>,<ax>,<ay>,<az>,<wx>,<wy>,<wz>
  GNSS,<ns>,<lat_deg>,<lon_deg>,<alt_m>,<ve>,<vn>,<vu>,<pos_sigma_m>,<vel_sigma_mps>

Usage:
  python3 tools/gen_synthetic_dataset.py [output_path]
"""

import math
import sys
import random

# ── Trajectory parameters ──────────────────────────────────────────────────────
ORIGIN_LAT_DEG = 47.37689          # ETH Zürich
ORIGIN_LON_DEG = 8.54169
ORIGIN_ALT_M   = 500.0

RADIUS_M       = 50.0              # circle radius
CIRCLE_PERIOD_S = 60.0             # one full lap
DURATION_S     = 120.0             # total recording time

IMU_HZ         = 100               # 100 Hz IMU
GNSS_HZ        = 1                 # 1 Hz GPS

GRAVITY        = 9.80665           # m/s²

# ── Reference ellipsoid (WGS84) ────────────────────────────────────────────────
A = 6_378_137.0
E2 = 6.69437999014e-3

def lla_to_ecef(lat_r, lon_r, alt):
    N = A / math.sqrt(1.0 - E2 * math.sin(lat_r)**2)
    x = (N + alt) * math.cos(lat_r) * math.cos(lon_r)
    y = (N + alt) * math.cos(lat_r) * math.sin(lon_r)
    z = (N * (1.0 - E2) + alt) * math.sin(lat_r)
    return x, y, z

def meters_to_deg_lat(m):
    return math.degrees(m / 6_378_137.0)

def meters_to_deg_lon(m, lat_deg):
    lat_r = math.radians(lat_deg)
    return math.degrees(m / (6_378_137.0 * math.cos(lat_r)))

# ── Trajectory: circular motion in ENU ────────────────────────────────────────
# Position in ENU:
#   e(t) = R * sin(ω*t)
#   n(t) = R * cos(ω*t) - R   (starts at origin heading north)
# Velocity ENU:
#   ve = R*ω * cos(ω*t)
#   vn = -R*ω * sin(ω*t)
# Heading (yaw from North, clockwise positive):
#   yaw = atan2(ve, vn)  (vehicle nose in direction of velocity)
# Body frame = NED-rotated by yaw (flat, level vehicle).
# Accelerometer (specific force in body frame):
#   Centripetal in ENU: ac_e = -R*ω² * sin(ω*t), ac_n = -R*ω² * cos(ω*t)
#   Transform ENU → body (level, yaw rotation only):
#     body x (forward) = ac_n * cos(yaw) + ac_e * sin(yaw)  ← flipped sign because NED
#     ...
#   Gravity in body (flat vehicle → gravity is straight down in NED → +Z in NED → body bz)
#   Specific force = measured accel − gravity_body → actually accel reads a = f + g_body
#   For a level vehicle with g_body = [0,0,g] (NED convention, z down):
#     ax_body = centripetal_forward
#     ay_body = centripetal_right
#     az_body = g  (gravity component when stationary is +g in NED z-down)
# Gyro (angular rate in body frame):
#   Only yaw rate ≠ 0: wz = -ω (right-hand rule, z-down NED → CW turn = negative)

omega = 2.0 * math.pi / CIRCLE_PERIOD_S   # rad/s
speed = RADIUS_M * omega                   # m/s ≈ 5.24 m/s

def trajectory(t):
    """Returns ENU position, ENU velocity, yaw_rad, centripetal ENU accel."""
    angle = omega * t
    e = RADIUS_M * math.sin(angle)
    n = RADIUS_M * (math.cos(angle) - 1.0)

    ve =  RADIUS_M * omega * math.cos(angle)
    vn = -RADIUS_M * omega * math.sin(angle)

    yaw = math.atan2(ve, vn)   # heading: angle of velocity from north

    # Centripetal acceleration in ENU (towards circle centre)
    ac_e = -RADIUS_M * omega**2 * math.sin(angle)
    ac_n = -RADIUS_M * omega**2 * math.cos(angle)

    return e, n, ve, vn, yaw, ac_e, ac_n

def enu_accel_to_body_ned(ac_e, ac_n, yaw):
    """
    Convert centripetal ENU [ac_e, ac_n, 0] into body-frame specific force.
    Body frame: x=forward (north-aligned when yaw=0), y=right, z=down (NED).
    Specific force f = a_body − (−g_body) = a_body + g_body.
    For level flight: g_body = [0, 0, g].
    """
    # Rotate ENU [ac_e, ac_n] into NED [ac_n, ac_e] then rotate by yaw
    # NED: x=north, y=east, z=down
    ac_north = ac_n
    ac_east  = ac_e

    # Body x (forward) and y (right) via yaw rotation (around NED z-down)
    ax = ac_north * math.cos(yaw) + ac_east * math.sin(yaw)
    ay = -ac_north * math.sin(yaw) + ac_east * math.cos(yaw)
    az = GRAVITY   # gravity reaction in z-down body

    return ax, ay, az

# ── Add gentle noise ──────────────────────────────────────────────────────────
rng = random.Random(42)

def noise(sigma):
    return rng.gauss(0.0, sigma)

def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/falconguide_synthetic.csv"

    imu_dt_ns  = int(1e9 / IMU_HZ)
    gnss_dt_ns = int(1e9 / GNSS_HZ)
    total_ns   = int(DURATION_S * 1e9)

    lat0 = math.radians(ORIGIN_LAT_DEG)
    lon0 = math.radians(ORIGIN_LON_DEG)

    lines = []
    lines.append("# FalconGuide synthetic dataset — circular trajectory")
    lines.append(f"# Origin: {ORIGIN_LAT_DEG:.6f}°N  {ORIGIN_LON_DEG:.6f}°E  alt={ORIGIN_ALT_M}m")
    lines.append(f"# Radius={RADIUS_M}m  speed={speed:.2f}m/s  duration={DURATION_S}s")

    next_imu_ns  = 0
    next_gnss_ns = 0

    while next_imu_ns <= total_ns or next_gnss_ns <= total_ns:
        do_imu  = next_imu_ns  <= total_ns and next_imu_ns  <= next_gnss_ns
        do_gnss = next_gnss_ns <= total_ns and (next_gnss_ns < next_imu_ns or not do_imu)

        if do_imu:
            ns = next_imu_ns
            t  = ns / 1e9
            _e, _n, _ve, _vn, yaw, ac_e, ac_n = trajectory(t)
            ax, ay, az = enu_accel_to_body_ned(ac_e, ac_n, yaw)
            wz = -omega + noise(0.001)   # yaw rate (z-down NED → negative for CCW in ENU)

            ax += noise(0.02)
            ay += noise(0.02)
            az += noise(0.02)
            wx = noise(0.001)
            wy = noise(0.001)

            lines.append(f"IMU,{ns},{ax:.6f},{ay:.6f},{az:.6f},{wx:.6f},{wy:.6f},{wz:.6f}")
            next_imu_ns += imu_dt_ns

        elif do_gnss:
            ns = next_gnss_ns
            t  = ns / 1e9
            e, n, ve, vn, _yaw, _ac_e, _ac_n = trajectory(t)

            # ENU offset → lat/lon
            lat_deg = ORIGIN_LAT_DEG + meters_to_deg_lat(n)
            lon_deg = ORIGIN_LON_DEG + meters_to_deg_lon(e, ORIGIN_LAT_DEG)
            alt_m   = ORIGIN_ALT_M + noise(0.5)

            ve += noise(0.1)
            vn += noise(0.1)

            pos_sigma = 1.5    # m
            vel_sigma = 0.2    # m/s

            lines.append(
                f"GNSS,{ns},{lat_deg:.9f},{lon_deg:.9f},{alt_m:.3f},"
                f"{ve:.4f},{vn:.4f},0.0000,{pos_sigma:.2f},{vel_sigma:.2f}"
            )
            next_gnss_ns += gnss_dt_ns
        else:
            break

    with open(out_path, "w") as f:
        f.write("\n".join(lines) + "\n")

    imu_count  = sum(1 for l in lines if l.startswith("IMU"))
    gnss_count = sum(1 for l in lines if l.startswith("GNSS"))
    print(f"Written {out_path}")
    print(f"  IMU rows : {imu_count}  ({imu_count/IMU_HZ:.1f} s at {IMU_HZ} Hz)")
    print(f"  GNSS rows: {gnss_count}  ({gnss_count/GNSS_HZ:.1f} s at {GNSS_HZ} Hz)")
    print(f"  Speed    : {speed:.2f} m/s  |  Radius: {RADIUS_M} m  |  Lap: {CIRCLE_PERIOD_S} s")

if __name__ == "__main__":
    main()
