#include "falconguide/estimation/backends/ekf/measurement_models/barometer.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/ekf_update.hpp"

#include <cmath>
#include <variant>

namespace falconguide::estimation::ekf {

static constexpr double kIsaExponent = 1.0 / 5.25588;

static double PressureToAltitude(double pressure_pa, double sea_level_pa) {
  return 44330.77 * (1.0 - std::pow(pressure_pa / sea_level_pa, kIsaExponent));
}

Barometer::Barometer(BarometerOptions options) : options_(std::move(options)) {}

bool Barometer::CanHandle(const SensorMeasurement& measurement) const {
  return std::holds_alternative<core::BarometerMeasurement>(measurement);
}

EstimatorUpdateResult Barometer::Apply(
    NominalState& nominal,
    Eigen::VectorXd& error_state,
    Eigen::MatrixXd& covariance,
    const StateLayout& layout,
    const SensorMeasurement& measurement,
    const UpdateContext& /*context*/) {

  const auto& baro = std::get<core::BarometerMeasurement>(measurement);
  if (baro.validity != core::MeasurementValidity::Valid) return EstimatorUpdateResult::Rejected;

  // ── Extract altitude observation ──────────────────────────────────────────
  double z_obs;
  double sigma_m;
  if (baro.altitude_m) {
    z_obs = *baro.altitude_m;
    sigma_m = options_.altitude_sigma_m.value_or(1.0);
  } else if (baro.pressure_pa > 0.0) {
    z_obs = PressureToAltitude(baro.pressure_pa, options_.isa_sea_level_pressure_pa);
    sigma_m = options_.altitude_sigma_m.value_or(5.0);
  } else {
    return EstimatorUpdateResult::Rejected;
  }

  const int n = layout.TotalSize();
  const auto& pos_seg = layout.Get(StateSegmentId::Position);

  // ── Predicted altitude ────────────────────────────────────────────────────
  double h_pred = nominal.position_enu_m.eigen().z();
  if (layout.Has(StateSegmentId::BaroBias)) {
    const auto& bb_seg = layout.Get(StateSegmentId::BaroBias);
    h_pred += error_state(bb_seg.offset);
  }

  // ── H matrix (1×n) ───────────────────────────────────────────────────────
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, n);
  H(0, pos_seg.offset + 2) = 1.0;
  if (layout.Has(StateSegmentId::BaroBias)) {
    H(0, layout.Get(StateSegmentId::BaroBias).offset) = 1.0;
  }

  const Eigen::VectorXd dz = Eigen::VectorXd::Constant(1, z_obs - h_pred);
  const Eigen::MatrixXd R_meas = Eigen::MatrixXd::Constant(1, 1, sigma_m * sigma_m);

  return EkfUpdate(error_state, covariance, H, dz, R_meas, options_.innovation_gate);
}

}  // namespace falconguide::estimation::ekf
