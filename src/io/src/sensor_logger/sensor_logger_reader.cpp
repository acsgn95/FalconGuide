#include "falconguide/io/sensor_logger/sensor_logger_reader.hpp"

#include "falconguide/core/coordinates.hpp"
#include "falconguide/core/sensors/gnss.hpp"
#include "falconguide/core/sensors/imu.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace falconguide::io {

namespace fs = std::filesystem;

// ── Helpers ───────────────────────────────────────────────────────────────────

static std::vector<std::string> Split(const std::string& line, char d = ',') {
    std::vector<std::string> t;
    std::istringstream ss(line);
    std::string tok;
    while (std::getline(ss, tok, d)) t.push_back(tok);
    return t;
}

static core::Timestamp MakeTs(int64_t ns) {
    core::Timestamp t;
    t.has_steady = true;
    t.steady = core::MonotonicTime(std::chrono::nanoseconds(ns));
    return t;
}

// Sensor Logger stores columns as: time, seconds_elapsed, z, y, x
static std::vector<std::pair<int64_t, std::array<double, 3>>> LoadXyz(const std::string& path) {
    std::vector<std::pair<int64_t, std::array<double, 3>>> rows;
    std::ifstream f(path);
    if (!f) return rows;
    std::string line;
    std::getline(f, line);  // skip header
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        auto t = Split(line);
        if (t.size() < 5) continue;
        int64_t ns = std::stoll(t[0]);
        double z = std::stod(t[2]);
        double y = std::stod(t[3]);
        double x = std::stod(t[4]);
        rows.push_back({ns, {x, y, z}});
    }
    return rows;
}

// ── Loaders ───────────────────────────────────────────────────────────────────

std::vector<SensorLoggerReader::RawImu> SensorLoggerReader::LoadImu(const std::string& root) {
    auto accel = LoadXyz(root + "/Accelerometer.csv");
    auto gyro = LoadXyz(root + "/Gyroscope.csv");

    // Sort both by timestamp
    auto cmp = [](const auto& a, const auto& b) { return a.first < b.first; };
    std::sort(accel.begin(), accel.end(), cmp);
    std::sort(gyro.begin(), gyro.end(), cmp);

    std::vector<RawImu> rows;
    rows.reserve(accel.size());

    // For each accel sample, interpolate gyro at the same timestamp
    std::size_t gi = 0;
    for (auto& [ns, a] : accel) {
        // Advance gyro pointer to the closest sample
        while (gi + 1 < gyro.size() && std::abs(gyro[gi + 1].first - ns) <= std::abs(gyro[gi].first - ns)) {
            ++gi;
        }

        RawImu imu;
        imu.ns = ns;
        imu.ax = a[0];
        imu.ay = a[1];
        imu.az = a[2];

        if (!gyro.empty()) {
            imu.wx = gyro[gi].second[0];
            imu.wy = gyro[gi].second[1];
            imu.wz = gyro[gi].second[2];
        }
        rows.push_back(imu);
    }
    return rows;
}

std::vector<SensorLoggerReader::RawGnss> SensorLoggerReader::LoadGnss(const std::string& root) {
    std::vector<RawGnss> rows;
    std::ifstream f(root + "/LocationGps.csv");
    if (!f) return rows;

    std::string line;
    std::getline(f, line);  // header: time,seconds_elapsed,bearingAccuracy,speedAccuracy,
                            //         verticalAccuracy,horizontalAccuracy,speed,bearing,altitude,longitude,latitude
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        auto t = Split(line);
        if (t.size() < 11) continue;

        RawGnss g;
        g.ns = std::stoll(t[0]);
        g.h_acc_m = std::stod(t[5]);
        g.v_acc_m = std::stod(t[4]);
        g.speed_mps = std::stod(t[6]);
        g.bearing_deg = std::stod(t[7]);
        g.alt_m = std::stod(t[8]);
        g.lon_deg = std::stod(t[9]);
        g.lat_deg = std::stod(t[10]);
        rows.push_back(g);
    }
    return rows;
}

std::vector<SensorLoggerReader::CameraFrame> SensorLoggerReader::LoadCameraFrames(const std::string& root) {
    std::vector<CameraFrame> frames;
    fs::path cam_dir = fs::path(root) / "Camera";
    if (!fs::exists(cam_dir)) return frames;

    for (auto& entry : fs::directory_iterator(cam_dir)) {
        if (entry.path().extension() != ".jpg" && entry.path().extension() != ".jpeg") continue;
        const std::string stem = entry.path().stem().string();
        try {
            int64_t ms = std::stoll(stem);
            frames.push_back({ms * 1'000'000LL, entry.path().string()});
        } catch (...) {
            continue;
        }
    }
    std::sort(frames.begin(), frames.end(), [](const auto& a, const auto& b) { return a.ns < b.ns; });
    return frames;
}

// ── Lifecycle ─────────────────────────────────────────────────────────────────

SensorLoggerReader::SensorLoggerReader(std::string root_dir) : root_(std::move(root_dir)) {}

std::string SensorLoggerReader::Name() const { return "SensorLoggerReader(" + root_ + ")"; }

ReaderCapabilities SensorLoggerReader::Capabilities() const {
    return ReaderCapabilities::Imu | ReaderCapabilities::Gnss | ReaderCapabilities::Camera;
}

bool SensorLoggerReader::Open() {
    imu_rows_ = LoadImu(root_);
    gnss_rows_ = LoadGnss(root_);
    frames_ = LoadCameraFrames(root_);

    if (imu_rows_.empty() && gnss_rows_.empty()) return false;

    // Build priority queue
    while (!pq_.empty()) pq_.pop();

    for (std::size_t i = 0; i < imu_rows_.size(); ++i) pq_.push({imu_rows_[i].ns, Event::Kind::Imu, i});
    for (std::size_t i = 0; i < gnss_rows_.size(); ++i) pq_.push({gnss_rows_[i].ns, Event::Kind::Gnss, i});
    for (std::size_t i = 0; i < frames_.size(); ++i) pq_.push({frames_[i].ns, Event::Kind::Camera, i});

    open_ = true;
    return true;
}

void SensorLoggerReader::Close() {
    open_ = false;
    while (!pq_.empty()) pq_.pop();
}

bool SensorLoggerReader::IsOpen() const { return open_; }

// ── Next ──────────────────────────────────────────────────────────────────────

ReadOutcome SensorLoggerReader::Next() {
    if (!open_ || pq_.empty()) return {ReadResult::EndOfData};

    const auto ev = pq_.top();
    pq_.pop();

    if (ev.kind == Event::Kind::Imu) {
        const auto& row = imu_rows_[ev.idx];

        core::ImuMeasurement imu;
        imu.timestamp = MakeTs(row.ns);
        imu.specific_force_mps2 = {row.ax, row.ay, row.az};
        imu.angular_rate_radps = {row.wx, row.wy, row.wz};
        imu.validity = core::MeasurementValidity::Valid;
        return {ReadResult::Ok, imu};
    }

    if (ev.kind == Event::Kind::Gnss) {
        const auto& row = gnss_rows_[ev.idx];

        constexpr double kDeg2Rad = M_PI / 180.0;
        core::Lla lla{row.lat_deg * kDeg2Rad, row.lon_deg * kDeg2Rad, row.alt_m};

        core::GnssSolution gnss;
        gnss.timestamp = MakeTs(row.ns);
        gnss.position_ecef_m = core::LlaToEcef(lla);
        gnss.fix_type = core::GnssFixType::Single;
        gnss.validity = core::MeasurementValidity::Valid;

        // Speed + bearing → ECEF velocity via EcefToEnuRotation (R_ecef_to_enu)
        double vn = row.speed_mps * std::cos(row.bearing_deg * kDeg2Rad);
        double ve = row.speed_mps * std::sin(row.bearing_deg * kDeg2Rad);

        // R_ecef_to_enu transposed gives R_enu_to_ecef
        Eigen::Matrix3d R_ecef_to_enu = core::EcefToEnuRotation(lla.latitude_rad, lla.longitude_rad);
        gnss.velocity_ecef_mps = core::Vec3<core::EcefFrame>(R_ecef_to_enu.transpose() * Eigen::Vector3d(ve, vn, 0.0));

        double ps = std::max(row.h_acc_m, 0.1);
        double vs = 0.5;  // speed accuracy not always reliable, use fixed 0.5 m/s
        gnss.position_covariance_ecef_m2 = Eigen::Matrix3d::Identity() * (ps * ps);
        gnss.velocity_covariance_ecef_m2ps2 = Eigen::Matrix3d::Identity() * (vs * vs);
        return {ReadResult::Ok, gnss};
    }

    if (ev.kind == Event::Kind::Camera) {
        // Emit a camera-only outcome — no SensorMeasurement, but carry the path
        ReadOutcome out;
        out.result = ReadResult::Ok;
        out.camera_frame_path = frames_[ev.idx].path;
        return out;
    }

    return {ReadResult::EndOfData};
}

}  // namespace falconguide::io
