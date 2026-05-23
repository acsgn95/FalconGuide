#include "falconguide/app/dataset_reader.hpp"
#include "falconguide/io/csv/csv_measurement_reader.hpp"
#include "falconguide/io/sensor_logger/sensor_logger_reader.hpp"

#include <stdexcept>

namespace falconguide::app {

// ── CsvDatasetReader ──────────────────────────────────────────────────────────

class CsvDatasetReader final : public IDatasetReader {
   public:
    explicit CsvDatasetReader(std::string path) : path_(std::move(path)), reader_(path_, "CsvDatasetReader") {}

    bool Open() override { return reader_.Open(); }
    void Close() override { reader_.Close(); }

    bool Rewind() override {
        reader_.Close();
        return reader_.Open();
    }

    io::ReadOutcome Next() override {
        auto outcome = reader_.Next();
        if (outcome.result == io::ReadResult::Ok) ++count_;
        return outcome;
    }

    std::string Name() const override { return "CsvDatasetReader(" + path_ + ")"; }
    std::size_t MeasurementsRead() const override { return count_; }

   private:
    std::string path_;
    io::CsvMeasurementReader reader_;
    std::size_t count_{0};
};

// ── SensorLoggerDatasetReader ─────────────────────────────────────────────────

class SensorLoggerDatasetReader final : public IDatasetReader {
   public:
    explicit SensorLoggerDatasetReader(std::string root) : root_(std::move(root)), reader_(root_) {}

    bool Open() override { return reader_.Open(); }
    void Close() override { reader_.Close(); }

    bool Rewind() override {
        reader_.Close();
        return reader_.Open();
    }

    io::ReadOutcome Next() override {
        auto out = reader_.Next();
        if (out.result == io::ReadResult::Ok) ++count_;
        return out;
    }

    std::string Name() const override { return "SensorLoggerDatasetReader(" + root_ + ")"; }
    std::size_t MeasurementsRead() const override { return count_; }

   private:
    std::string root_;
    io::SensorLoggerReader reader_;
    std::size_t count_{0};
};

// ── Factory ───────────────────────────────────────────────────────────────────

std::unique_ptr<IDatasetReader> MakeDatasetReader(const DatasetConfig& cfg) {
    if (cfg.type == "csv") {
        return std::make_unique<CsvDatasetReader>(cfg.path);
    }
    if (cfg.type == "sensor_logger") {
        return std::make_unique<SensorLoggerDatasetReader>(cfg.path);
    }
    throw std::invalid_argument("Unknown dataset type: " + cfg.type);
}

}  // namespace falconguide::app
