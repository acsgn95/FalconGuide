#pragma once

/**
 * @file sensor_logger_reader.hpp
 * @brief Dataset reader for the Sensor Logger Android app.
 *
 * Reads the directory layout produced by Sensor Logger:
 *   <root>/Accelerometer.csv
 *   <root>/Gyroscope.csv
 *   <root>/LocationGps.csv
 *   <root>/Camera/<timestamp_ms>.jpg
 *
 * Measurements are emitted in ascending timestamp order.
 * IMU samples from the two files are paired by nearest-neighbor interpolation.
 */

#include "falconguide/io/measurement_reader.hpp"

#include <queue>
#include <string>
#include <vector>

namespace falconguide::io {

class SensorLoggerReader final : public IMeasurementReader {
   public:
    explicit SensorLoggerReader(std::string root_dir);

    [[nodiscard]] std::string Name() const override;
    [[nodiscard]] ReaderCapabilities Capabilities() const override;

    bool Open() override;
    void Close() override;
    [[nodiscard]] bool IsOpen() const override;

    ReadOutcome Next() override;

   private:
    struct RawImu {
        int64_t ns{0};
        double ax{0}, ay{0}, az{0};
        double wx{0}, wy{0}, wz{0};
    };

    struct RawGnss {
        int64_t ns{0};
        double lat_deg{0}, lon_deg{0}, alt_m{0};
        double speed_mps{0}, bearing_deg{0};
        double h_acc_m{0}, v_acc_m{0};
    };

    struct CameraFrame {
        int64_t ns{0};
        std::string path;
    };

    struct Event {
        int64_t ns{0};
        enum class Kind { Imu, Gnss, Camera } kind{};
        std::size_t idx{0};

        bool operator>(const Event& o) const { return ns > o.ns; }
    };

    static std::vector<RawImu> LoadImu(const std::string& root);
    static std::vector<RawGnss> LoadGnss(const std::string& root);
    static std::vector<CameraFrame> LoadCameraFrames(const std::string& root);

    std::string root_;
    bool open_{false};

    std::vector<RawImu> imu_rows_;
    std::vector<RawGnss> gnss_rows_;
    std::vector<CameraFrame> frames_;

    std::priority_queue<Event, std::vector<Event>, std::greater<Event>> pq_;
};

}  // namespace falconguide::io
