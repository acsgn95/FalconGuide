#pragma once

#include "falconguide/estimation/backends/ekf/measurement_models/measurement_model.hpp"

#include <cstddef>

namespace falconguide::estimation::ekf {

// ── GnssTightlyCoupledOptions ─────────────────────────────────────────────────
struct GnssTightlyCoupledOptions {
  // Pseudorange noise floor (m, 1-sigma) used when CN0 is unavailable.
  // When CN0 is provided the per-SV noise is scaled as:
  //   σ_ρ = base_pseudorange_sigma_m * 10^(-CN0_dBHz / 40) + noise_floor_m
  double base_pseudorange_sigma_m{3.0};
  double pseudorange_noise_floor_m{0.1};

  // Doppler noise (m/s, 1-sigma).
  double base_doppler_sigma_mps{0.5};

  // Per-observation innovation gate in units of innovation sigma.
  // 0 means no gating.  3.0 corresponds to ≈99.7% acceptance for Gaussian noise.
  double innovation_gate_sigma{3.0};

  // Minimum number of healthy SVs required to apply an update.
  std::size_t min_satellites{4};
};

// ── GnssTightlyCoupled ────────────────────────────────────────────────────────
//
// Fuses raw GNSS pseudoranges and Doppler observations
// (GnssTightlyCoupledEpoch) directly into the EKF error state, bypassing the
// receiver's navigation solution.
//
// Observation models
// ------------------
// Pseudorange for satellite i:
//   ρᵢ = |pˢᵢ − pᵣ| + c·δt_b − c·δtˢᵢ + Δᵢon + Δᵢtr + εᵢ_ρ
//        ↑ geometric   ↑ rcvr   ↑ sat clk   ↑ atmo   ↑ noise
//
// Sagnac correction (Earth-rotation during signal travel):
//   ρˢᵃᵍⁿᵃᶜ = (Ωₑ/c) · (xˢ·yᵣ − yˢ·xᵣ)
//
// Linearised (δ = observation − predicted):
//   δρᵢ = −eᵢᵀ R_{ENU→ECEF} δp + c δt_b + εᵢ_ρ
//
// Doppler for satellite i (range-rate):
//   ρ̇ᵢ = eᵢᵀ (vˢᵢ − vᵣ) + c·δt_d − c·δṫˢᵢ + εᵢ_ρ̇
//
// Linearised:
//   δρ̇ᵢ = −eᵢᵀ R_{ENU→ECEF} δv + c δt_d + εᵢ_ρ̇
//
// H rows are placed at the correct offsets in the n-dimensional error state
// using the StateLayout, so the observation model works regardless of which
// optional state segments are active.
//
// Update strategy: sequential scalar updates per SV — each observation (and
// Doppler if present) is processed one at a time, updating P between SVs.
// This is numerically equivalent to the batch update and avoids inverting a
// large innovation covariance matrix.
//
// Requirements:
//   - StateSegmentId::GnssClock must be registered (enable_gnss_clock_state).
//   - UpdateContext::ltp must be set (estimator initialised from GNSS).
//
class GnssTightlyCoupled : public IMeasurementModel {
 public:
  explicit GnssTightlyCoupled(GnssTightlyCoupledOptions options = {});

  [[nodiscard]] bool CanHandle(const SensorMeasurement& measurement) const override;

  EstimatorUpdateResult Apply(
      NominalState& nominal,
      Eigen::VectorXd& error_state,
      Eigen::MatrixXd& covariance,
      const StateLayout& layout,
      const SensorMeasurement& measurement,
      const UpdateContext& context) override;

 private:
  // Apply a single scalar Kalman update in-place.
  // Returns true if the observation passed the innovation gate.
  bool ScalarUpdate(
      Eigen::VectorXd& error_state,
      Eigen::MatrixXd& covariance,
      const Eigen::VectorXd& H_row,
      double innovation,
      double sigma) const;

  // Returns the carrier frequency (Hz) for a given signal type.
  static double CarrierFrequencyHz(core::GnssSignal signal);

  // Sagnac (Earth-rotation) correction to geometric range.
  static double SagnacCorrectionM(
      const Eigen::Vector3d& pos_sv_ecef_m,
      const Eigen::Vector3d& pos_rcv_ecef_m);

  GnssTightlyCoupledOptions options_;
};

}  // namespace falconguide::estimation::ekf
