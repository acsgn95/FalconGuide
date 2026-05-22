#pragma once

/**
 * @file measurement_helpers.hpp
 * @brief Generic helpers for measurement timestamp and validity access.
 */

#include "falconguide/core/sensors/common.hpp"
#include "falconguide/core/time.hpp"
#include "falconguide/core/time_helpers.hpp"

namespace falconguide::core {

/// @brief Returns the timestamp member from a measurement type.
/// @tparam Measurement Measurement type with a public timestamp field.
template <typename Measurement>
[[nodiscard]] const Timestamp &GetTimestamp(const Measurement &measurement) {
    return measurement.timestamp;
}

/// @brief Returns the validity member from a measurement type.
/// @tparam Measurement Measurement type with a public validity field.
template <typename Measurement>
[[nodiscard]] MeasurementValidity GetValidity(const Measurement &measurement) {
    return measurement.validity;
}

/// @brief Returns true when a measurement validity flag is Valid.
template <typename Measurement>
[[nodiscard]] bool IsValid(const Measurement &measurement) {
    return GetValidity(measurement) == MeasurementValidity::Valid;
}

/// @brief Checks whether @p current is timestamped before @p previous.
template <typename Measurement>
[[nodiscard]] bool IsOutOfOrder(const Measurement &previous, const Measurement &current) {
    const Timestamp &previous_timestamp = GetTimestamp(previous);
    const Timestamp &current_timestamp = GetTimestamp(current);
    return IsBefore(current_timestamp, previous_timestamp);
}

}  // namespace falconguide::core
