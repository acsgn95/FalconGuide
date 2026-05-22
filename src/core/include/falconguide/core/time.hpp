#pragma once

/**
 * @file time.hpp
 * @brief Time primitives used by measurements and navigation states.
 */

#include <chrono>
#include <cstdint>

namespace falconguide::core {

/// @brief Signed duration stored internally as nanoseconds.
class Duration {
   public:
    /// @brief Creates a zero duration.
    constexpr Duration() = default;
    /// @brief Creates a duration from nanoseconds.
    explicit constexpr Duration(std::chrono::nanoseconds value) : value_(value) {}

    /// @brief Creates a duration from seconds.
    /// @param seconds Duration in seconds.
    /// @return Duration rounded to integer nanoseconds.
    [[nodiscard]] static constexpr Duration FromSeconds(double seconds) {
        return Duration(std::chrono::nanoseconds(static_cast<std::int64_t>(seconds * 1'000'000'000.0)));
    }

    /// @brief Returns the duration as nanoseconds.
    [[nodiscard]] constexpr std::chrono::nanoseconds nanoseconds() const { return value_; }
    /// @brief Returns the duration as floating-point seconds.
    [[nodiscard]] double seconds() const { return static_cast<double>(value_.count()) / 1'000'000'000.0; }

   private:
    std::chrono::nanoseconds value_{0};
};

/// @brief Monotonic timestamp suitable for ordering local sensor samples.
class MonotonicTime {
   public:
    /// @brief Creates a zero monotonic timestamp.
    constexpr MonotonicTime() = default;
    /// @brief Creates a monotonic timestamp from nanoseconds since an arbitrary
    /// epoch.
    explicit constexpr MonotonicTime(std::chrono::nanoseconds value) : value_(value) {}

    /// @brief Returns nanoseconds since the monotonic epoch.
    [[nodiscard]] constexpr std::chrono::nanoseconds nanoseconds_since_epoch() const { return value_; }

    /// @brief Computes the signed duration between two monotonic timestamps.
    [[nodiscard]] constexpr Duration operator-(const MonotonicTime &other) const {
        return Duration(value_ - other.value_);
    }

   private:
    std::chrono::nanoseconds value_{0};
};

/// @brief GPS week plus seconds-of-week timestamp.
class GpsTime {
   public:
    /// @brief Creates GPS week zero at second zero.
    constexpr GpsTime() = default;
    /// @brief Creates a GPS timestamp.
    constexpr GpsTime(std::int32_t week, double seconds_of_week) : week_(week), seconds_of_week_(seconds_of_week) {}

    /// @brief Returns the GPS week number.
    [[nodiscard]] constexpr std::int32_t week() const { return week_; }
    /// @brief Returns seconds elapsed within the GPS week.
    [[nodiscard]] constexpr double seconds_of_week() const { return seconds_of_week_; }

   private:
    std::int32_t week_{0};
    double seconds_of_week_{0.0};
};

/// @brief Measurement timestamp that may carry monotonic time, GPS time, or
/// both.
struct Timestamp {
    MonotonicTime steady;    ///< Monotonic clock value when available.
    GpsTime gps;             ///< GPS time value when available.
    bool has_steady{false};  ///< True when @ref steady is valid.
    bool has_gps{false};     ///< True when @ref gps is valid.
};

}  // namespace falconguide::core
