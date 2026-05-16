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

}  // namespace falconguide::core
