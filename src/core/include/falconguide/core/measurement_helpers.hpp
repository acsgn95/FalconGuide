#pragma once

#include "falconguide/core/sensors/common.hpp"
#include "falconguide/core/time.hpp"

namespace falconguide::core {

template <typename Measurement>
[[nodiscard]] const Timestamp& GetTimestamp(const Measurement& measurement) {
  return measurement.timestamp;
}

template <typename Measurement>
[[nodiscard]] MeasurementValidity GetValidity(const Measurement& measurement) {
  return measurement.validity;
}

template <typename Measurement>
[[nodiscard]] bool IsValid(const Measurement& measurement) {
  return GetValidity(measurement) == MeasurementValidity::Valid;
}

template <typename Measurement>
[[nodiscard]] bool IsOutOfOrder(const Measurement& previous, const Measurement& current) {
  const Timestamp& previous_timestamp = GetTimestamp(previous);
  const Timestamp& current_timestamp = GetTimestamp(current);

  if (previous_timestamp.has_steady && current_timestamp.has_steady) {
    return current_timestamp.steady.nanoseconds_since_epoch() < previous_timestamp.steady.nanoseconds_since_epoch();
  }

  if (previous_timestamp.has_gps && current_timestamp.has_gps) {
    if (current_timestamp.gps.week() != previous_timestamp.gps.week()) {
      return current_timestamp.gps.week() < previous_timestamp.gps.week();
    }
    return current_timestamp.gps.seconds_of_week() < previous_timestamp.gps.seconds_of_week();
  }

  return false;
}

}  // namespace falconguide::core
