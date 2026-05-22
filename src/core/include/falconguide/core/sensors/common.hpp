#pragma once

/**
 * @file common.hpp
 * @brief Shared sensor measurement metadata and noise descriptors.
 */

namespace falconguide::core {

/// @brief Validity flag carried by all sensor measurement structs.
enum class MeasurementValidity {
    Valid,       ///< Measurement passed source-level validation.
    Saturated,   ///< Sensor saturated or clipped.
    Dropped,     ///< Source explicitly reports a dropped sample.
    OutOfOrder,  ///< Timestamp order is invalid for the producing stream.
    Unknown      ///< Validity is not known.
};

/// @brief Continuous sensor noise density and bias random walk terms.
struct NoiseDensity {
    double continuous_noise_density{0.0};  ///< White-noise density in sensor-specific units per sqrt(Hz).
    double random_walk{0.0};               ///< Bias random-walk density in sensor-specific
                                           ///< units per sqrt(second).
};

}  // namespace falconguide::core
