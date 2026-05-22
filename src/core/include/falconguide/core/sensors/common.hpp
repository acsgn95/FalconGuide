#pragma once

namespace falconguide::core {

enum class MeasurementValidity {
  Valid,
  Saturated,
  Dropped,
  OutOfOrder,
  Unknown
};

struct NoiseDensity {
  double continuous_noise_density{0.0};
  double random_walk{0.0};
};

}  // namespace falconguide::core
