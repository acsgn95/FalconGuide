#pragma once

/**
 * @file measurement_variant_helpers.hpp
 * @brief Helpers for visiting and inspecting SensorMeasurement variants.
 */

#include "falconguide/core/measurement_helpers.hpp"
#include "falconguide/estimation/estimator_interface.hpp"

#include <utility>
#include <variant>

namespace falconguide::estimation {

/// @brief Visits a SensorMeasurement with a forwarding visitor.
/// @tparam Visitor Callable accepted by std::visit.
template <typename Visitor>
decltype(auto) VisitMeasurement(const SensorMeasurement &measurement,
                                Visitor &&visitor) {
  return std::visit(std::forward<Visitor>(visitor), measurement);
}

/// @brief Returns the timestamp of the active measurement variant.
[[nodiscard]] inline const core::Timestamp &
GetTimestamp(const SensorMeasurement &measurement) {
  return std::visit(
      [](const auto &typed_measurement) -> const core::Timestamp & {
        return core::GetTimestamp(typed_measurement);
      },
      measurement);
}

/// @brief Returns the validity of the active measurement variant.
[[nodiscard]] inline core::MeasurementValidity
GetValidity(const SensorMeasurement &measurement) {
  return std::visit(
      [](const auto &typed_measurement) {
        return core::GetValidity(typed_measurement);
      },
      measurement);
}

/// @brief Returns true when the active measurement variant is valid.
[[nodiscard]] inline bool IsValid(const SensorMeasurement &measurement) {
  return GetValidity(measurement) == core::MeasurementValidity::Valid;
}

} // namespace falconguide::estimation
