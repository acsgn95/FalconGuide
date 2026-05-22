#pragma once

/**
 * @file time_helpers.hpp
 * @brief Ordering and arithmetic helpers for FalconGuide timestamps.
 */

#include "falconguide/core/time.hpp"

#include <optional>

namespace falconguide::core {

/// @brief Result of comparing two timestamps.
enum class TimestampOrdering {
  Before,       ///< Left timestamp is earlier than the right timestamp.
  Equal,        ///< Timestamps represent the same time.
  After,        ///< Left timestamp is later than the right timestamp.
  NotComparable ///< No common valid time basis is available.
};

/// @brief Compares two GPS timestamps.
/// @return Relative ordering of @p lhs against @p rhs.
[[nodiscard]] inline TimestampOrdering CompareGpsTime(const GpsTime &lhs,
                                                      const GpsTime &rhs) {
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

/// @brief Compares timestamps using monotonic time first, then GPS time.
/// @return NotComparable if neither timestamp pair shares a valid time basis.
[[nodiscard]] inline TimestampOrdering CompareTimestamps(const Timestamp &lhs,
                                                         const Timestamp &rhs) {
  if (lhs.has_steady && rhs.has_steady) {
    if (lhs.steady.nanoseconds_since_epoch() <
        rhs.steady.nanoseconds_since_epoch()) {
      return TimestampOrdering::Before;
    }
    if (lhs.steady.nanoseconds_since_epoch() >
        rhs.steady.nanoseconds_since_epoch()) {
      return TimestampOrdering::After;
    }
    return TimestampOrdering::Equal;
  }

  if (lhs.has_gps && rhs.has_gps) {
    return CompareGpsTime(lhs.gps, rhs.gps);
  }

  return TimestampOrdering::NotComparable;
}

/// @brief Returns true when @p lhs is strictly before @p rhs.
[[nodiscard]] inline bool IsBefore(const Timestamp &lhs, const Timestamp &rhs) {
  return CompareTimestamps(lhs, rhs) == TimestampOrdering::Before;
}

/// @brief Returns true when @p lhs is strictly after @p rhs.
[[nodiscard]] inline bool IsAfter(const Timestamp &lhs, const Timestamp &rhs) {
  return CompareTimestamps(lhs, rhs) == TimestampOrdering::After;
}

/// @brief Returns true when both timestamps compare equal.
[[nodiscard]] inline bool IsEqualTime(const Timestamp &lhs,
                                      const Timestamp &rhs) {
  return CompareTimestamps(lhs, rhs) == TimestampOrdering::Equal;
}

/// @brief Computes lhs - rhs when timestamps share a comparable time basis.
/// @return Signed duration or std::nullopt when timestamps are not comparable.
[[nodiscard]] inline std::optional<Duration>
TimeDifference(const Timestamp &lhs, const Timestamp &rhs) {
  if (lhs.has_steady && rhs.has_steady) {
    return lhs.steady - rhs.steady;
  }

  if (lhs.has_gps && rhs.has_gps && lhs.gps.week() == rhs.gps.week()) {
    return Duration::FromSeconds(lhs.gps.seconds_of_week() -
                                 rhs.gps.seconds_of_week());
  }

  return std::nullopt;
}

} // namespace falconguide::core
