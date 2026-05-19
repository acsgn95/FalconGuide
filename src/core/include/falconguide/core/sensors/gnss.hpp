#pragma once

#include "falconguide/core/frames.hpp"
#include "falconguide/core/sensors/common.hpp"
#include "falconguide/core/time.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <optional>
#include <vector>

namespace falconguide::core {

enum class GnssConstellation {
  Gps,
  Glonass,
  Galileo,
  BeiDou,
  Qzss,
  Sbas,
  Unknown
};

enum class GnssSignal {
  L1,
  L2,
  L5,
  E1,
  E5a,
  E5b,
  B1,
  B2,
  Unknown
};

enum class GnssFixType {
  NoFix,
  Single,
  Differential,
  RtkFloat,
  RtkFixed,
  PrecisePointPositioning
};

struct SatelliteId {
  GnssConstellation constellation{GnssConstellation::Unknown};
  std::uint16_t prn{0};
};

struct GnssRawObservation {
  SatelliteId satellite;
  GnssSignal signal{GnssSignal::Unknown};
  double pseudorange_m{0.0};
  std::optional<double> carrier_phase_cycles;
  std::optional<double> doppler_hz;
  std::optional<double> carrier_to_noise_density_dbhz;
  std::optional<double> lock_time_s;
  MeasurementValidity validity{MeasurementValidity::Valid};
};

struct GnssObservationEpoch {
  Timestamp timestamp;
  std::vector<GnssRawObservation> observations;
  MeasurementValidity validity{MeasurementValidity::Valid};
};

struct GnssSolution {
  Timestamp timestamp;
  Vec3<EcefFrame> position_ecef_m;
  Vec3<EcefFrame> velocity_ecef_mps;
  Eigen::Matrix3d position_covariance_ecef_m2{Eigen::Matrix3d::Zero()};
  Eigen::Matrix3d velocity_covariance_ecef_m2ps2{Eigen::Matrix3d::Zero()};
  GnssFixType fix_type{GnssFixType::NoFix};
  std::optional<double> horizontal_dop;
  std::optional<double> vertical_dop;
  MeasurementValidity validity{MeasurementValidity::Valid};
};

struct GnssAntennaCalibration {
  Transform<GnssAntennaFrame, BodyFrame> antenna_to_body;
  Vec3<GnssAntennaFrame> phase_center_offset_m;
};

// ── SatelliteState ────────────────────────────────────────────────────────────
//
// Pre-computed satellite position, velocity, and clock correction at signal
// transmission time, in ECEF (WGS84).  Computed by the GNSS driver from
// broadcast or precise ephemeris before packaging a GnssTightlyCoupledEpoch.
//
struct SatelliteState {
  SatelliteId satellite;

  // ECEF position and velocity at transmission time (Earth-fixed frame).
  Vec3<EcefFrame> position_ecef_m;
  Vec3<EcefFrame> velocity_ecef_mps;

  // Satellite clock correction already applied in the pseudorange by some
  // receivers; stored here so the estimator can verify or re-apply.
  //   corrected_pseudorange = raw_pseudorange + c * clock_bias_s
  double clock_bias_s{0.0};    // satellite clock bias (s)
  double clock_drift_sps{0.0}; // satellite clock drift (s/s)

  // Atmospheric delays computed by the driver (Klobuchar / Saastamoinen
  // models or SBAS corrections).  Zero means no correction applied.
  double ionospheric_delay_m{0.0};
  double tropospheric_delay_m{0.0};

  // Set false if the navigation message marks this SV unhealthy.
  bool healthy{true};
};

// ── GnssTightlyCoupledObservation ─────────────────────────────────────────────
//
// Pairs a raw pseudorange / Doppler observation with the corresponding
// satellite state so the estimator never has to look them up separately.
//
struct GnssTightlyCoupledObservation {
  GnssRawObservation raw;
  SatelliteState     satellite;
};

// ── GnssTightlyCoupledEpoch ───────────────────────────────────────────────────
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
  Timestamp timestamp;
  std::vector<GnssTightlyCoupledObservation> sv_observations;
  MeasurementValidity validity{MeasurementValidity::Valid};
};

}  // namespace falconguide::core
