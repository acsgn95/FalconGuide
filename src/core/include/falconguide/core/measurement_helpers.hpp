#pragma once

#include "falconguide/core/sensors/common.hpp"
#include "falconguide/core/time.hpp"
#include "falconguide/core/time_helpers.hpp"

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
  return IsBefore(current_timestamp, previous_timestamp);
}

}  // namespace falconguide::core
