#include "falconguide/estimation/backends/ekf/ekf_estimator.hpp"

#include "falconguide/core/coordinates.hpp"
#include "falconguide/core/math.hpp"
#include "falconguide/core/time_helpers.hpp"
#include "falconguide/estimation/measurement_variant_helpers.hpp"

#include <variant>

namespace falconguide::estimation::ekf {

// ── Constants ─────────────────────────────────────────────────────────────────

static constexpr double kGravityMps2    = 9.80665;
static constexpr double kInitPosSigmaM  = 10.0;   // initial position (overridden by GNSS cov when available)
static constexpr double kInitVelSigmaMps = 1.0;
static constexpr double kInitAttSigmaRad = 0.5;   // ~30° attitude uncertainty at init
static constexpr double kInitAbaSigma   = 0.3;    // m/s²
static constexpr double kInitGbaSigma   = 0.05;   // rad/s
static constexpr double kInitClkBiasSigmaM  = 100.0; // m  (loosely constrained)
static constexpr double kInitClkDriftSigmaMps = 10.0; // m/s

// ── Constructor ───────────────────────────────────────────────────────────────

EkfEstimator::EkfEstimator(EkfOptions options)
    : options_(std::move(options)),
      imu_buffer_(/*max_size=*/2000) {
  state_.layout    = BuildLayout();
  state_.error_state = state_.layout.ZeroVector();
  state_.covariance  = state_.layout.ZeroMatrix();
}

// ── Public API ────────────────────────────────────────────────────────────────

void EkfEstimator::RegisterMeasurementModel(std::unique_ptr<IMeasurementModel> model) {
  measurement_models_.push_back(std::move(model));
}

EstimatorInfo EkfEstimator::Info() const {
  return {EstimatorBackend::Ekf, "EkfEstimator", "1.0"};
}

const EstimatorOptions& EkfEstimator::Options() const {
  return options_.base;
}

bool EkfEstimator::IsInitialized() const {
  std::unique_lock lock(mutex_);
  return initialised_;
}

std::optional<core::NavigationState> EkfEstimator::LatestState() const {
  std::unique_lock lock(mutex_);
  if (!initialised_) return std::nullopt;
  return BuildNavigationState();
}

void EkfEstimator::Reset() {
  std::unique_lock lock(mutex_);
  initialised_           = false;
  local_tangent_plane_   = std::nullopt;
  last_aiding_timestamp_ = std::nullopt;
  imu_buffer_.Clear();
  state_.layout      = BuildLayout();
  state_.error_state = state_.layout.ZeroVector();
  state_.covariance  = state_.layout.ZeroMatrix();
  state_.nominal     = NominalState{};
}

EstimatorUpdateResult EkfEstimator::AddMeasurement(const SensorMeasurement& measurement) {
  std::unique_lock lock(mutex_);

  // ── IMU: buffer for propagation ────────────────────────────────────────────
  if (const auto* imu = std::get_if<core::ImuMeasurement>(&measurement)) {
    const auto result = imu_buffer_.Insert(*imu);
    if (result == core::BufferInsertResult::RejectedOutOfOrder ||
        result == core::BufferInsertResult::RejectedNotComparable) {
      return EstimatorUpdateResult::OutOfOrder;
    }
    return EstimatorUpdateResult::Buffered;
  }

  // ── First GNSS: initialise ────────────────────────────────────────────────
  if (!initialised_) {
    if (const auto* gnss = std::get_if<core::GnssSolution>(&measurement)) {
      return TryInitialiseFromGnss(*gnss) ? EstimatorUpdateResult::Accepted
                                          : EstimatorUpdateResult::Rejected;
    }
    return EstimatorUpdateResult::NotInitialized;
  }

  // ── Propagate IMU buffer to measurement time ───────────────────────────────
  PropagateTo(estimation::GetTimestamp(measurement));

  // ── Dispatch to measurement models ────────────────────────────────────────
  const UpdateContext ctx{local_tangent_plane_ ? &*local_tangent_plane_ : nullptr};

  for (auto& model : measurement_models_) {
    if (!model->CanHandle(measurement)) continue;

    const auto result = model->Apply(
        state_.nominal, state_.error_state, state_.covariance,
        state_.layout, measurement, ctx);

    if (result == EstimatorUpdateResult::Accepted) {
      InjectErrorAndReset();
      last_aiding_timestamp_ = estimation::GetTimestamp(measurement);
    }
    return result;
  }

  return EstimatorUpdateResult::Rejected;
}

EstimatorUpdateResult EkfEstimator::ProcessUntil(const core::Timestamp& timestamp) {
  std::unique_lock lock(mutex_);
  if (!initialised_) return EstimatorUpdateResult::NotInitialized;
  PropagateTo(timestamp);
  return EstimatorUpdateResult::Accepted;
}

// ── Layout ────────────────────────────────────────────────────────────────────

StateLayout EkfEstimator::BuildLayout() const {
  StateLayout layout = MakeStandardLayout();
  if (options_.enable_gnss_clock_state) layout.Register(StateSegmentId::GnssClock, 2);
  if (options_.enable_baro_bias_state)  layout.Register(StateSegmentId::BaroBias,  1);
  return layout;
}

// ── IMU Propagation ───────────────────────────────────────────────────────────
//
// Error-state INS mechanisation in the ENU local-tangent-plane frame.
//
// Nominal state update:
//   p   ← p + v·dt + ½ a_enu·dt²
//   v   ← v + a_enu·dt          where a_enu = R·f + g_enu
//   q   ← q ⊗ Exp(ω·dt)
//
// Error covariance: P ← Φ P Φᵀ + Q
//   Φ = I + F·dt  (first-order van Loan discretisation)
//   Q = G·Qc·Gᵀ·dt
//
// F (continuous, n×n):
//   δṗ =          δv
//   δv̇ = −R[f×]δθ  −R δba
//   δθ̇ = −[ω×]δθ      −δbg
//   δḃa = 0 (random walk driven by Qc)
//   δḃg = 0 (random walk driven by Qc)
//   (clock drift drives clock bias when GnssClock is active)
//
void EkfEstimator::PropagateImu(const core::ImuMeasurement& imu, const double dt_s) {
  using namespace core;
  using Eigen::Matrix3d;
  using Eigen::MatrixXd;
  using Eigen::Vector3d;

  auto& nom = state_.nominal;
  const auto& layout = state_.layout;

  // Bias-corrected IMU readings
  const Vector3d f     = imu.specific_force_mps2.eigen() - nom.accel_bias_mps2.eigen();
  const Vector3d omega = imu.angular_rate_radps.eigen()  - nom.gyro_bias_radps.eigen();

  const Matrix3d R = nom.orientation_body_to_enu.toRotationMatrix();
  const Vector3d g_enu(0.0, 0.0, -kGravityMps2);
  const Vector3d a_enu = R * f + g_enu;

  // ── Nominal state ─────────────────────────────────────────────────────────
  nom.position_enu_m   = Vec3<EnuFrame>(nom.position_enu_m.eigen() +
                                        nom.velocity_enu_mps.eigen() * dt_s +
                                        0.5 * a_enu * dt_s * dt_s);
  nom.velocity_enu_mps = Vec3<EnuFrame>(nom.velocity_enu_mps.eigen() + a_enu * dt_s);
  nom.orientation_body_to_enu =
      (nom.orientation_body_to_enu * ExpMapSo3(omega * dt_s)).normalized();
  nom.timestamp = imu.timestamp;

  // ── Error covariance ──────────────────────────────────────────────────────
  const int n       = layout.TotalSize();
  const int n_noise = 12;  // [n_a(3), n_g(3), n_ba(3), n_bg(3)]

  const auto& pos_seg = layout.Get(StateSegmentId::Position);
  const auto& vel_seg = layout.Get(StateSegmentId::Velocity);
  const auto& att_seg = layout.Get(StateSegmentId::Attitude);
  const auto& aba_seg = layout.Get(StateSegmentId::AccelBias);
  const auto& gba_seg = layout.Get(StateSegmentId::GyroBias);

  MatrixXd F = MatrixXd::Zero(n, n);

  // δṗ = δv
  F.block(pos_seg.offset, vel_seg.offset, 3, 3) = Matrix3d::Identity();

  // δv̇ = −R[f×]δθ − R δba
  F.block(vel_seg.offset, att_seg.offset, 3, 3) = -R * SkewSymmetric(f);
  F.block(vel_seg.offset, aba_seg.offset, 3, 3) = -R;

  // δθ̇ = −[ω×]δθ − δbg
  F.block(att_seg.offset, att_seg.offset, 3, 3) = -SkewSymmetric(omega);
  F.block(att_seg.offset, gba_seg.offset, 3, 3) = -Matrix3d::Identity();

  // Clock: bias driven by drift  δ̇t_b = δt_d
  if (layout.Has(StateSegmentId::GnssClock)) {
    const auto& clk = layout.Get(StateSegmentId::GnssClock);
    F(clk.offset, clk.offset + 1) = 1.0;
  }

  MatrixXd G = MatrixXd::Zero(n, n_noise);
  G.block(vel_seg.offset, 0, 3, 3) = -R;                // accel noise  → δv̇
  G.block(att_seg.offset, 3, 3, 3) = -Matrix3d::Identity(); // gyro noise   → δθ̇
  G.block(aba_seg.offset, 6, 3, 3) =  Matrix3d::Identity(); // accel RW     → δḃa
  G.block(gba_seg.offset, 9, 3, 3) =  Matrix3d::Identity(); // gyro RW      → δḃg

  const double qa  = options_.imu_noise.accel_noise_density_mps2_per_sqrthz;
  const double qg  = options_.imu_noise.gyro_noise_density_radps_per_sqrthz;
  const double qba = options_.imu_noise.accel_random_walk_mps3_per_sqrthz;
  const double qbg = options_.imu_noise.gyro_random_walk_radps2_per_sqrthz;

  MatrixXd Qc = MatrixXd::Zero(n_noise, n_noise);
  Qc.block<3, 3>(0, 0) = Matrix3d::Identity() * (qa  * qa);
  Qc.block<3, 3>(3, 3) = Matrix3d::Identity() * (qg  * qg);
  Qc.block<3, 3>(6, 6) = Matrix3d::Identity() * (qba * qba);
  Qc.block<3, 3>(9, 9) = Matrix3d::Identity() * (qbg * qbg);

  const MatrixXd Phi = MatrixXd::Identity(n, n) + F * dt_s;
  const MatrixXd Q   = G * Qc * G.transpose() * dt_s;

  state_.covariance = Phi * state_.covariance * Phi.transpose() + Q;
  state_.covariance = core::SymmetrizeCovariance(state_.covariance);
}

void EkfEstimator::PropagateTo(const core::Timestamp& target) {
  const auto imu_batch = imu_buffer_.PopUntil(target);
  for (const auto& imu : imu_batch) {
    if (!state_.nominal.timestamp.has_steady && !state_.nominal.timestamp.has_gps) {
      state_.nominal.timestamp = imu.timestamp;
      continue;
    }
    const auto dt_opt = core::TimeDifference(imu.timestamp, state_.nominal.timestamp);
    if (!dt_opt) continue;
    const double dt_s = dt_opt->seconds();
    if (dt_s < options_.min_imu_dt_s) continue;

    // Split large gaps to limit linearisation error.
    double remaining = dt_s;
    while (remaining > 0.0) {
      const double step = std::min(remaining, options_.max_imu_dt_s);
      PropagateImu(imu, step);
      remaining -= step;
    }
    state_.nominal.timestamp = imu.timestamp;
  }
}

// ── Error injection ───────────────────────────────────────────────────────────
//
// Folds the accumulated error state into the nominal state, then zeroes δx.
//
//   p  ← p + δp
//   v  ← v + δv
//   q  ← q ⊗ Exp(δθ)    (right-perturbation on SO(3))
//   ba ← ba + δba
//   bg ← bg + δbg
//   [t_b, t_d] ← [t_b, t_d] + [δt_b, δt_d]   (if GnssClock active)
//
void EkfEstimator::InjectErrorAndReset() {
  using namespace core;
  auto& nom              = state_.nominal;
  const auto& layout     = state_.layout;
  const Eigen::VectorXd& dx = state_.error_state;

  const auto& pos = layout.Get(StateSegmentId::Position);
  nom.position_enu_m = Vec3<EnuFrame>(nom.position_enu_m.eigen() + dx.segment(pos.offset, pos.size));

  const auto& vel = layout.Get(StateSegmentId::Velocity);
  nom.velocity_enu_mps = Vec3<EnuFrame>(nom.velocity_enu_mps.eigen() + dx.segment(vel.offset, vel.size));

  const auto& att = layout.Get(StateSegmentId::Attitude);
  nom.orientation_body_to_enu =
      (nom.orientation_body_to_enu * ExpMapSo3(dx.segment(att.offset, att.size))).normalized();

  const auto& aba = layout.Get(StateSegmentId::AccelBias);
  nom.accel_bias_mps2 = Vec3<ImuFrame>(nom.accel_bias_mps2.eigen() + dx.segment(aba.offset, aba.size));

  const auto& gba = layout.Get(StateSegmentId::GyroBias);
  nom.gyro_bias_radps = Vec3<ImuFrame>(nom.gyro_bias_radps.eigen() + dx.segment(gba.offset, gba.size));

  if (layout.Has(StateSegmentId::GnssClock) && nom.gnss_clock) {
    const auto& clk = layout.Get(StateSegmentId::GnssClock);
    *nom.gnss_clock += dx.segment(clk.offset, clk.size);
  }

  if (layout.Has(StateSegmentId::BaroBias)) {
    // BaroBias is injected by the barometer update model directly into the
    // nominal state via this mechanism when that model is implemented.
  }

  state_.error_state.setZero();
}

// ── Initialisation ────────────────────────────────────────────────────────────

bool EkfEstimator::TryInitialiseFromGnss(const core::GnssSolution& gnss) {
  if (gnss.fix_type == core::GnssFixType::NoFix)             return false;
  if (gnss.validity != core::MeasurementValidity::Valid)     return false;

  // ── ENU origin at first fix ───────────────────────────────────────────────
  const core::Lla origin_lla = core::EcefToLla(gnss.position_ecef_m);
  local_tangent_plane_ = core::LocalTangentPlane(origin_lla);
  const Eigen::Matrix3d R_ecef_to_enu = local_tangent_plane_->ecef_to_enu_rotation();

  // ── Nominal state ─────────────────────────────────────────────────────────
  auto& nom           = state_.nominal;
  nom.timestamp       = gnss.timestamp;
  nom.position_enu_m  = core::Vec3<core::EnuFrame>(0.0, 0.0, 0.0);  // at origin by definition
  nom.velocity_enu_mps = core::Vec3<core::EnuFrame>(R_ecef_to_enu * gnss.velocity_ecef_mps.eigen());
  nom.orientation_body_to_enu = Eigen::Quaterniond::Identity();  // level + north — yaw unknown
  nom.accel_bias_mps2 = core::Vec3<core::ImuFrame>(0.0, 0.0, 0.0);
  nom.gyro_bias_radps = core::Vec3<core::ImuFrame>(0.0, 0.0, 0.0);

  if (state_.layout.Has(StateSegmentId::GnssClock)) {
    nom.gnss_clock = Eigen::Vector2d::Zero();
  }

  // ── Initial covariance ────────────────────────────────────────────────────
  const int n            = state_.layout.TotalSize();
  state_.error_state     = Eigen::VectorXd::Zero(n);
  state_.covariance      = Eigen::MatrixXd::Zero(n, n);

  const auto& pos = state_.layout.Get(StateSegmentId::Position);
  const auto& vel = state_.layout.Get(StateSegmentId::Velocity);
  const auto& att = state_.layout.Get(StateSegmentId::Attitude);
  const auto& aba = state_.layout.Get(StateSegmentId::AccelBias);
  const auto& gba = state_.layout.Get(StateSegmentId::GyroBias);

  // Use GNSS covariance (rotated to ENU) for position and velocity.
  state_.covariance.block(pos.offset, pos.offset, 3, 3) =
      R_ecef_to_enu * gnss.position_covariance_ecef_m2 * R_ecef_to_enu.transpose();
  state_.covariance.block(vel.offset, vel.offset, 3, 3) =
      R_ecef_to_enu * gnss.velocity_covariance_ecef_m2ps2 * R_ecef_to_enu.transpose();

  // Fall back to constants if the GNSS covariance is zero (e.g. simulated data).
  if (state_.covariance.block(pos.offset, pos.offset, 3, 3).norm() < 1e-12) {
    state_.covariance.block(pos.offset, pos.offset, 3, 3) =
        Eigen::Matrix3d::Identity() * (kInitPosSigmaM * kInitPosSigmaM);
    state_.covariance.block(vel.offset, vel.offset, 3, 3) =
        Eigen::Matrix3d::Identity() * (kInitVelSigmaMps * kInitVelSigmaMps);
  }

  state_.covariance.block(att.offset, att.offset, 3, 3) =
      Eigen::Matrix3d::Identity() * (kInitAttSigmaRad * kInitAttSigmaRad);
  state_.covariance.block(aba.offset, aba.offset, 3, 3) =
      Eigen::Matrix3d::Identity() * (kInitAbaSigma * kInitAbaSigma);
  state_.covariance.block(gba.offset, gba.offset, 3, 3) =
      Eigen::Matrix3d::Identity() * (kInitGbaSigma * kInitGbaSigma);

  if (state_.layout.Has(StateSegmentId::GnssClock)) {
    const auto& clk = state_.layout.Get(StateSegmentId::GnssClock);
    state_.covariance(clk.offset,     clk.offset)     = kInitClkBiasSigmaM  * kInitClkBiasSigmaM;
    state_.covariance(clk.offset + 1, clk.offset + 1) = kInitClkDriftSigmaMps * kInitClkDriftSigmaMps;
  }

  initialised_ = true;
  return true;
}

// ── NavigationState output ────────────────────────────────────────────────────

core::NavigationState EkfEstimator::BuildNavigationState() const {
  using namespace core;
  const auto& nom    = state_.nominal;
  const auto& layout = state_.layout;

  NavigationState out;
  out.timestamp             = nom.timestamp;
  out.position_enu_m        = nom.position_enu_m;
  out.velocity_enu_mps      = nom.velocity_enu_mps;
  out.orientation_body_to_enu = nom.orientation_body_to_enu;

  if (local_tangent_plane_) {
    out.position_ecef_m  = local_tangent_plane_->EnuToEcef(nom.position_enu_m);
    out.velocity_ecef_mps = Vec3<EcefFrame>(
        local_tangent_plane_->ecef_to_enu_rotation().transpose() * nom.velocity_enu_mps.eigen());
  }

  // ── Extract core 15×15 covariance block ───────────────────────────────────
  // The standard segments are always at indices [0..14] in MakeStandardLayout().
  static constexpr int kCoreDim = 15;
  if (layout.TotalSize() >= kCoreDim) {
    out.covariance = state_.covariance.block<kCoreDim, kCoreDim>(0, 0);
  }

  // ── Quality ───────────────────────────────────────────────────────────────
  out.quality.initialized = initialised_;
  if (initialised_) {
    const auto& pos = layout.Get(StateSegmentId::Position);
    const auto& vel = layout.Get(StateSegmentId::Velocity);
    const auto& att = layout.Get(StateSegmentId::Attitude);
    out.quality.position_accuracy_m =
        std::sqrt(state_.covariance.block(pos.offset, pos.offset, 3, 3).trace() / 3.0);
    out.quality.velocity_accuracy_mps =
        std::sqrt(state_.covariance.block(vel.offset, vel.offset, 3, 3).trace() / 3.0);
    out.quality.attitude_accuracy_rad =
        std::sqrt(state_.covariance.block(att.offset, att.offset, 3, 3).trace() / 3.0);
  }

  // ── Status ────────────────────────────────────────────────────────────────
  if (!initialised_) {
    out.status = NavigationStatus::NotInitialized;
    out.mode   = EstimatorMode::Unknown;
    return out;
  }

  out.mode = measurement_models_.empty() ? EstimatorMode::InertialOnly
                                         : EstimatorMode::MultiSensorFusion;

  if (last_aiding_timestamp_ && nom.timestamp.has_steady && last_aiding_timestamp_->has_steady) {
    const auto age = TimeDifference(nom.timestamp, *last_aiding_timestamp_);
    if (age && age->seconds() > options_.dead_reckoning_threshold_s) {
      out.status = NavigationStatus::DeadReckoning;
    } else {
      out.status = NavigationStatus::Nominal;
    }
  } else {
    out.status = NavigationStatus::DeadReckoning;
  }

  // IMU sensor status
  out.sensors.imu.health           = SensorHealth::Healthy;
  out.sensors.imu.used_in_solution = true;

  return out;
}

}  // namespace falconguide::estimation::ekf
