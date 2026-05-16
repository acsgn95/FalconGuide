#pragma once

#include <chrono>
#include <cstdint>

namespace falconguide::core {

class Duration {
 public:
  constexpr Duration() = default;
  explicit constexpr Duration(std::chrono::nanoseconds value) : value_(value) {}

  [[nodiscard]] static constexpr Duration FromSeconds(double seconds) {
    return Duration(std::chrono::nanoseconds(static_cast<std::int64_t>(seconds * 1'000'000'000.0)));
  }

  [[nodiscard]] constexpr std::chrono::nanoseconds nanoseconds() const { return value_; }
  [[nodiscard]] double seconds() const { return static_cast<double>(value_.count()) / 1'000'000'000.0; }

 private:
  std::chrono::nanoseconds value_{0};
};

class MonotonicTime {
 public:
  constexpr MonotonicTime() = default;
  explicit constexpr MonotonicTime(std::chrono::nanoseconds value) : value_(value) {}

  [[nodiscard]] constexpr std::chrono::nanoseconds nanoseconds_since_epoch() const { return value_; }

  [[nodiscard]] constexpr Duration operator-(const MonotonicTime& other) const {
    return Duration(value_ - other.value_);
  }

 private:
  std::chrono::nanoseconds value_{0};
};

class GpsTime {
 public:
  constexpr GpsTime() = default;
  constexpr GpsTime(std::int32_t week, double seconds_of_week) : week_(week), seconds_of_week_(seconds_of_week) {}

  [[nodiscard]] constexpr std::int32_t week() const { return week_; }
  [[nodiscard]] constexpr double seconds_of_week() const { return seconds_of_week_; }

 private:
  std::int32_t week_{0};
  double seconds_of_week_{0.0};
};

struct Timestamp {
  MonotonicTime steady;
  GpsTime gps;
  bool has_steady{false};
  bool has_gps{false};
};

}  // namespace falconguide::core
