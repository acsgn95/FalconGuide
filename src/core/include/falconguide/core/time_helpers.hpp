#pragma once

#include "falconguide/core/time.hpp"

#include <optional>

namespace falconguide::core {

enum class TimestampOrdering {
  Before,
  Equal,
  After,
  NotComparable
};

[[nodiscard]] inline TimestampOrdering CompareGpsTime(const GpsTime& lhs, const GpsTime& rhs) {
  if (lhs.week() < rhs.week()) {
    return TimestampOrdering::Before;
  }
  if (lhs.week() > rhs.week()) {
    return TimestampOrdering::After;
  }
  if (lhs.seconds_of_week() < rhs.seconds_of_week()) {
    return TimestampOrdering::Before;
  }
  if (lhs.seconds_of_week() > rhs.seconds_of_week()) {
    return TimestampOrdering::After;
  }
  return TimestampOrdering::Equal;
}

[[nodiscard]] inline TimestampOrdering CompareTimestamps(const Timestamp& lhs, const Timestamp& rhs) {
  if (lhs.has_steady && rhs.has_steady) {
    if (lhs.steady.nanoseconds_since_epoch() < rhs.steady.nanoseconds_since_epoch()) {
      return TimestampOrdering::Before;
    }
    if (lhs.steady.nanoseconds_since_epoch() > rhs.steady.nanoseconds_since_epoch()) {
      return TimestampOrdering::After;
    }
    return TimestampOrdering::Equal;
  }

  if (lhs.has_gps && rhs.has_gps) {
    return CompareGpsTime(lhs.gps, rhs.gps);
  }

  return TimestampOrdering::NotComparable;
}

[[nodiscard]] inline bool IsBefore(const Timestamp& lhs, const Timestamp& rhs) {
  return CompareTimestamps(lhs, rhs) == TimestampOrdering::Before;
}

[[nodiscard]] inline bool IsAfter(const Timestamp& lhs, const Timestamp& rhs) {
  return CompareTimestamps(lhs, rhs) == TimestampOrdering::After;
}

[[nodiscard]] inline bool IsEqualTime(const Timestamp& lhs, const Timestamp& rhs) {
  return CompareTimestamps(lhs, rhs) == TimestampOrdering::Equal;
}

[[nodiscard]] inline std::optional<Duration> TimeDifference(const Timestamp& lhs, const Timestamp& rhs) {
  if (lhs.has_steady && rhs.has_steady) {
    return lhs.steady - rhs.steady;
  }

  if (lhs.has_gps && rhs.has_gps && lhs.gps.week() == rhs.gps.week()) {
    return Duration::FromSeconds(lhs.gps.seconds_of_week() - rhs.gps.seconds_of_week());
  }

  return std::nullopt;
}

}  // namespace falconguide::core
