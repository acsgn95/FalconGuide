#pragma once

#include "falconguide/core/measurement_helpers.hpp"
#include "falconguide/estimation/estimator_interface.hpp"

#include <utility>
#include <variant>

namespace falconguide::estimation {

template <typename Visitor>
decltype(auto) VisitMeasurement(const SensorMeasurement& measurement, Visitor&& visitor) {
  return std::visit(std::forward<Visitor>(visitor), measurement);
}

[[nodiscard]] inline const core::Timestamp& GetTimestamp(const SensorMeasurement& measurement) {
  return std::visit([](const auto& typed_measurement) -> const core::Timestamp& {
    return core::GetTimestamp(typed_measurement);
  }, measurement);
}

[[nodiscard]] inline core::MeasurementValidity GetValidity(const SensorMeasurement& measurement) {
  return std::visit([](const auto& typed_measurement) {
    return core::GetValidity(typed_measurement);
  }, measurement);
}

[[nodiscard]] inline bool IsValid(const SensorMeasurement& measurement) {
  return GetValidity(measurement) == core::MeasurementValidity::Valid;
}

}  // namespace falconguide::estimation
