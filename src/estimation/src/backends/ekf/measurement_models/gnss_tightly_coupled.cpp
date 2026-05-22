#include "falconguide/estimation/backends/ekf/measurement_models/gnss_tightly_coupled.hpp"

#include "falconguide/core/math.hpp"

#include <cmath>
#include <variant>

namespace falconguide::estimation::ekf {

// ── Physical constants ────────────────────────────────────────────────────────
static constexpr double kSpeedOfLightMps = 299'792'458.0;
static constexpr double kEarthRotationRps = 7.2921151467e-5;  // WGS84 Ωₑ (rad/s)

// ── Constructor ───────────────────────────────────────────────────────────────

GnssTightlyCoupled::GnssTightlyCoupled(GnssTightlyCoupledOptions options) : options_(std::move(options)) {}

// ── CanHandle ─────────────────────────────────────────────────────────────────

bool GnssTightlyCoupled::CanHandle(const SensorMeasurement& measurement) const {
    return std::holds_alternative<core::GnssTightlyCoupledEpoch>(measurement);
}

// ── CarrierFrequencyHz ────────────────────────────────────────────────────────

double GnssTightlyCoupled::CarrierFrequencyHz(core::GnssSignal signal) {
    switch (signal) {
        case core::GnssSignal::L1:
            return 1'575.42e6;
        case core::GnssSignal::L2:
            return 1'227.60e6;
        case core::GnssSignal::L5:
            return 1'176.45e6;
        case core::GnssSignal::E1:
            return 1'575.42e6;
        case core::GnssSignal::E5a:
            return 1'176.45e6;
        case core::GnssSignal::E5b:
            return 1'207.14e6;
        case core::GnssSignal::B1:
            return 1'561.098e6;
        case core::GnssSignal::B2:
            return 1'207.14e6;
        default:
            return 1'575.42e6;  // fallback to GPS L1
    }
}

// ── SagnacCorrectionM ─────────────────────────────────────────────────────────
//
// Accounts for Earth's rotation during signal propagation (Sagnac effect).
// Correction is typically 10–40 m and must not be ignored in TC GNSS.
//
//   ρˢᵃᵍ = (Ωₑ/c) · (xˢ·yᵣ − yˢ·xᵣ)
//
double GnssTightlyCoupled::SagnacCorrectionM(const Eigen::Vector3d& pos_sv, const Eigen::Vector3d& pos_rcv) {
    return (kEarthRotationRps / kSpeedOfLightMps) * (pos_sv.x() * pos_rcv.y() - pos_sv.y() * pos_rcv.x());
}

// ── ScalarUpdate ──────────────────────────────────────────────────────────────
//
// Single-observation EKF update (Joseph form for numerical stability).
// Returns false if the innovation fails the gate.
//
bool GnssTightlyCoupled::ScalarUpdate(Eigen::VectorXd& error_state, Eigen::MatrixXd& covariance,
                                      const Eigen::VectorXd& H_row, const double innovation, const double sigma) const {
    // Innovation variance
    const double S = H_row.dot(covariance * H_row) + sigma * sigma;
    if (S <= 0.0) return false;

    // Innovation gate
    if (options_.innovation_gate_sigma > 0.0 && std::abs(innovation) > options_.innovation_gate_sigma * std::sqrt(S)) {
        return false;
    }

    const Eigen::VectorXd K = (covariance * H_row) / S;
    error_state += K * innovation;

    // Joseph form: P = (I − KH)P(I − KH)ᵀ + K σ² Kᵀ
    const int n = static_cast<int>(error_state.size());
    const Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(n, n) - K * H_row.transpose();
    covariance = IKH * covariance * IKH.transpose() + K * K.transpose() * (sigma * sigma);
    covariance = core::SymmetrizeCovariance(covariance);
    return true;
}

// ── Apply ─────────────────────────────────────────────────────────────────────

EstimatorUpdateResult GnssTightlyCoupled::Apply(NominalState& nominal, Eigen::VectorXd& error_state,
                                                Eigen::MatrixXd& covariance, const StateLayout& layout,
                                                const SensorMeasurement& measurement, const UpdateContext& context) {
    // ── Pre-conditions ────────────────────────────────────────────────────────
    if (!layout.Has(StateSegmentId::GnssClock)) return EstimatorUpdateResult::Rejected;
    if (!context.ltp) return EstimatorUpdateResult::NotInitialized;
    if (!nominal.gnss_clock) return EstimatorUpdateResult::NotInitialized;

    const auto& epoch = std::get<core::GnssTightlyCoupledEpoch>(measurement);
    if (epoch.validity != core::MeasurementValidity::Valid) return EstimatorUpdateResult::Rejected;
    if (epoch.sv_observations.empty()) return EstimatorUpdateResult::Rejected;

    // ── Receiver state in ECEF ────────────────────────────────────────────────
    const Eigen::Vector3d pos_rcv_ecef = context.ltp->EnuToEcef(nominal.position_enu_m).eigen();
    const Eigen::Matrix3d R_enu_to_ecef = context.ltp->ecef_to_enu_rotation().transpose();
    const Eigen::Vector3d vel_rcv_ecef = R_enu_to_ecef * nominal.velocity_enu_mps.eigen();

    const double clk_bias_m = (*nominal.gnss_clock)(0);
    const double clk_drift_mps = (*nominal.gnss_clock)(1);

    const int n = layout.TotalSize();
    const auto& pos_seg = layout.Get(StateSegmentId::Position);
    const auto& vel_seg = layout.Get(StateSegmentId::Velocity);
    const auto& clk_seg = layout.Get(StateSegmentId::GnssClock);

    // ── Sequential update per satellite ───────────────────────────────────────
    std::size_t accepted = 0;

    for (const auto& sv_obs : epoch.sv_observations) {
        if (sv_obs.raw.validity != core::MeasurementValidity::Valid) continue;
        if (!sv_obs.satellite.healthy) continue;

        const Eigen::Vector3d p_sv = sv_obs.satellite.position_ecef_m.eigen();
        const Eigen::Vector3d v_sv = sv_obs.satellite.velocity_ecef_mps.eigen();

        // Geometric range and unit line-of-sight vector (receiver → satellite)
        const Eigen::Vector3d diff = p_sv - pos_rcv_ecef;
        const double range = diff.norm();
        if (range < 1e-3) continue;
        const Eigen::Vector3d e = diff / range;

        // ── Pseudorange update ─────────────────────────────────────────────────
        {
            // Predicted pseudorange
            const double sagnac = SagnacCorrectionM(p_sv, pos_rcv_ecef);
            const double rho_pred = range + sagnac + clk_bias_m - sv_obs.satellite.clock_bias_s * kSpeedOfLightMps +
                                    sv_obs.satellite.ionospheric_delay_m + sv_obs.satellite.tropospheric_delay_m;

            const double innovation = sv_obs.raw.pseudorange_m - rho_pred;

            // H row: δρ = −eᵀ R_{ENU→ECEF} δp + c δt_b
            // The position error is in ENU; we need the projection onto ECEF LOS.
            Eigen::VectorXd H_rho = Eigen::VectorXd::Zero(n);
            H_rho.segment(pos_seg.offset, 3) = -(e.transpose() * R_enu_to_ecef).transpose();
            H_rho(clk_seg.offset) = 1.0;  // receiver clock bias

            // Per-SV pseudorange noise (CN0-weighted if available)
            double sigma_rho = options_.base_pseudorange_sigma_m;
            if (sv_obs.raw.carrier_to_noise_density_dbhz) {
                const double cn0 = *sv_obs.raw.carrier_to_noise_density_dbhz;
                sigma_rho = options_.base_pseudorange_sigma_m * std::pow(10.0, -cn0 / 40.0) +
                            options_.pseudorange_noise_floor_m;
            }

            if (ScalarUpdate(error_state, covariance, H_rho, innovation, sigma_rho)) {
                ++accepted;
            }
        }

        // ── Doppler update (range-rate) ────────────────────────────────────────
        if (sv_obs.raw.doppler_hz) {
            const double f_carrier = CarrierFrequencyHz(sv_obs.raw.signal);

            // Convert Doppler frequency to range-rate (m/s).
            // Sign convention: positive Doppler → satellite approaching → range decreasing.
            const double range_rate_obs = -(*sv_obs.raw.doppler_hz) * kSpeedOfLightMps / f_carrier;

            // Predicted range-rate
            const double range_rate_pred =
                e.dot(v_sv - vel_rcv_ecef) + clk_drift_mps - sv_obs.satellite.clock_drift_sps * kSpeedOfLightMps;

            const double innovation = range_rate_obs - range_rate_pred;

            // H row: δρ̇ = −eᵀ R_{ENU→ECEF} δv + c δt_d
            Eigen::VectorXd H_rdot = Eigen::VectorXd::Zero(n);
            H_rdot.segment(vel_seg.offset, 3) = -(e.transpose() * R_enu_to_ecef).transpose();
            H_rdot(clk_seg.offset + 1) = 1.0;  // receiver clock drift

            ScalarUpdate(error_state, covariance, H_rdot, innovation, options_.base_doppler_sigma_mps);
        }
    }

    if (accepted < options_.min_satellites) return EstimatorUpdateResult::Rejected;
    return EstimatorUpdateResult::Accepted;
}

}  // namespace falconguide::estimation::ekf
