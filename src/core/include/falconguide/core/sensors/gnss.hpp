#pragma once

/**
 * @file gnss.hpp
 * @brief GNSS observation, solution, calibration, and tightly-coupled data
 * types.
 */

#include "falconguide/core/frames.hpp"
#include "falconguide/core/sensors/common.hpp"
#include "falconguide/core/time.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <optional>
#include <vector>

namespace falconguide::core {

/// @brief Satellite constellation identifier.
enum class GnssConstellation {
    Gps,      ///< GPS constellation.
    Glonass,  ///< GLONASS constellation.
    Galileo,  ///< Galileo constellation.
    BeiDou,   ///< BeiDou constellation.
    Qzss,     ///< QZSS regional constellation.
    Sbas,     ///< SBAS augmentation satellite.
    Unknown   ///< Unknown or unsupported constellation.
};

/// @brief GNSS carrier/signal band identifier.
enum class GnssSignal {
    L1,      ///< L1 signal.
    L2,      ///< L2 signal.
    L5,      ///< L5 signal.
    E1,      ///< Galileo E1 signal.
    E5a,     ///< Galileo E5a signal.
    E5b,     ///< Galileo E5b signal.
    B1,      ///< BeiDou B1 signal.
    B2,      ///< BeiDou B2 signal.
    Unknown  ///< Unknown or unsupported signal.
};

/// @brief Receiver-level GNSS solution quality.
enum class GnssFixType {
    NoFix,                   ///< No valid navigation fix.
    Single,                  ///< Standalone single-point fix.
    Differential,            ///< Differential GNSS fix.
    RtkFloat,                ///< RTK float ambiguity fix.
    RtkFixed,                ///< RTK fixed ambiguity fix.
    PrecisePointPositioning  ///< Precise point positioning fix.
};

/// @brief Satellite identifier within a constellation.
struct SatelliteId {
    GnssConstellation constellation{GnssConstellation::Unknown};  ///< Satellite constellation.
    std::uint16_t prn{0};                                         ///< PRN or constellation-specific satellite number.
};

/// @brief Raw per-satellite observation produced by a GNSS receiver.
struct GnssRawObservation {
    SatelliteId satellite;                                     ///< Observed satellite.
    GnssSignal signal{GnssSignal::Unknown};                    ///< Signal used for the observation.
    double pseudorange_m{0.0};                                 ///< Code pseudorange in meters.
    std::optional<double> carrier_phase_cycles;                ///< Optional carrier phase in cycles.
    std::optional<double> doppler_hz;                          ///< Optional Doppler shift in hertz.
    std::optional<double> carrier_to_noise_density_dbhz;       ///< Optional C/N0 in dB-Hz.
    std::optional<double> lock_time_s;                         ///< Optional continuous tracking lock time.
    MeasurementValidity validity{MeasurementValidity::Valid};  ///< Observation validity state.
};

/// @brief Timestamped batch of raw GNSS observations.
struct GnssObservationEpoch {
    Timestamp timestamp;                                       ///< Epoch timestamp.
    std::vector<GnssRawObservation> observations;              ///< Raw observations visible at this epoch.
    MeasurementValidity validity{MeasurementValidity::Valid};  ///< Epoch validity state.
};

/// @brief Receiver-computed navigation solution.
struct GnssSolution {
    Timestamp timestamp;                                                      ///< Solution timestamp.
    Vec3<EcefFrame> position_ecef_m;                                          ///< Position in ECEF meters.
    Vec3<EcefFrame> velocity_ecef_mps;                                        ///< Velocity in ECEF meters per second.
    Eigen::Matrix3d position_covariance_ecef_m2{Eigen::Matrix3d::Zero()};     ///< ECEF position covariance.
    Eigen::Matrix3d velocity_covariance_ecef_m2ps2{Eigen::Matrix3d::Zero()};  ///< ECEF velocity covariance.
    GnssFixType fix_type{GnssFixType::NoFix};                                 ///< Receiver fix type.
    std::optional<double> horizontal_dop;                      ///< Optional horizontal dilution of precision.
    std::optional<double> vertical_dop;                        ///< Optional vertical dilution of precision.
    MeasurementValidity validity{MeasurementValidity::Valid};  ///< Solution validity state.
};

/// @brief GNSS antenna lever-arm and phase-center calibration.
struct GnssAntennaCalibration {
    Transform<GnssAntennaFrame, BodyFrame> antenna_to_body;  ///< Antenna-frame to body-frame transform.
    Vec3<GnssAntennaFrame> phase_center_offset_m;            ///< Phase-center offset in antenna frame.
};

// ── SatelliteState
// ────────────────────────────────────────────────────────────
//
// Pre-computed satellite position, velocity, and clock correction at signal
// transmission time, in ECEF (WGS84).  Computed by the GNSS driver from
// broadcast or precise ephemeris before packaging a GnssTightlyCoupledEpoch.
//
struct SatelliteState {
    SatelliteId satellite;  ///< Satellite that this state describes.

    // ECEF position and velocity at transmission time (Earth-fixed frame).
    Vec3<EcefFrame> position_ecef_m;    ///< ECEF satellite position at transmit time.
    Vec3<EcefFrame> velocity_ecef_mps;  ///< ECEF satellite velocity at transmit time.

    // Satellite clock correction already applied in the pseudorange by some
    // receivers; stored here so the estimator can verify or re-apply.
    //   corrected_pseudorange = raw_pseudorange + c * clock_bias_s
    double clock_bias_s{0.0};     ///< Satellite clock bias in seconds.
    double clock_drift_sps{0.0};  ///< Satellite clock drift in seconds per second.

    // Atmospheric delays computed by the driver (Klobuchar / Saastamoinen
    // models or SBAS corrections).  Zero means no correction applied.
    double ionospheric_delay_m{0.0};   ///< Ionospheric delay correction in meters.
    double tropospheric_delay_m{0.0};  ///< Tropospheric delay correction in meters.

    // Set false if the navigation message marks this SV unhealthy.
    bool healthy{true};  ///< False when navigation data marks the satellite unhealthy.
};

// ── GnssTightlyCoupledObservation
// ─────────────────────────────────────────────
//
// Pairs a raw pseudorange / Doppler observation with the corresponding
// satellite state so the estimator never has to look them up separately.
//
struct GnssTightlyCoupledObservation {
    GnssRawObservation raw;    ///< Receiver raw observation.
    SatelliteState satellite;  ///< Satellite state matching the raw observation.
};

// ── GnssTightlyCoupledEpoch
// ───────────────────────────────────────────────────
//
// Complete tightly coupled measurement epoch: a set of paired raw observations
// and satellite states ready for direct fusion into the EKF error state.
//
// Workflow:
//   1. GNSS driver decodes ephemeris → SatelliteState
//   2. Driver packages GnssTightlyCoupledObservation per visible SV
//   3. GnssTightlyCoupled measurement model builds H, computes innovations,
//      applies sequential EKF updates for pseudorange and Doppler
//
struct GnssTightlyCoupledEpoch {
    Timestamp timestamp;                                         ///< Epoch timestamp.
    std::vector<GnssTightlyCoupledObservation> sv_observations;  ///< Paired observations and satellite states.
    MeasurementValidity validity{MeasurementValidity::Valid};    ///< Epoch validity state.
};

}  // namespace falconguide::core
