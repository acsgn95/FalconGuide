#include "falconguide/estimation/backends/ukf/ukf_estimator.hpp"

#include "falconguide/core/coordinates.hpp"
#include "falconguide/core/math.hpp"
#include "falconguide/core/time_helpers.hpp"
#include "falconguide/estimation/measurement_variant_helpers.hpp"
#include "falconguide/logger/logger.hpp"

#include <array>
#include <variant>

namespace falconguide::estimation::ukf {

// ── Physical constants ────────────────────────────────────────────────────────
static constexpr double kGravityMps2     = 9.80665;
static constexpr double kInitPosSigmaM   = 10.0;
static constexpr double kInitVelSigmaMps =  1.0;
static constexpr double kInitAttSigmaRad =  0.5;
static constexpr double kInitAbaSigma    =  0.3;
static constexpr double kInitGbaSigma    =  0.05;

// ── SigmaWeights::Compute ─────────────────────────────────────────────────────

SigmaWeights SigmaWeights::Compute(const MerweSigmaParams& p) noexcept {
  SigmaWeights w;
  const double n      = kStateSize;
  const double lambda = p.alpha * p.alpha * (n + p.kappa) - n;

  w.mean_weights(0) = lambda / (n + lambda);
  w.cov_weights(0)  = lambda / (n + lambda) + (1.0 - p.alpha * p.alpha + p.beta);

  const double w_i = 1.0 / (2.0 * (n + lambda));
  for (int i = 1; i < kNumSigmaPoints; ++i) {
    w.mean_weights(i) = w_i;
    w.cov_weights(i)  = w_i;
  }
  return w;
}

// ── Constructor ───────────────────────────────────────────────────────────────

UkfEstimator::UkfEstimator(UkfOptions options)
    : options_(std::move(options)),
      weights_(SigmaWeights::Compute(options_.sigma_params)),
      imu_buffer_(2000) {}

// ── RegisterMeasurementModel ──────────────────────────────────────────────────

void UkfEstimator::RegisterMeasurementModel(std::unique_ptr<IUkfMeasurementModel> model) {
  measurement_models_.push_back(std::move(model));
}

// ── INavigationEstimator ──────────────────────────────────────────────────────

EstimatorInfo UkfEstimator::Info() const {
  return {EstimatorBackend::Custom, "UkfEstimator", "1.0"};
}

const EstimatorOptions& UkfEstimator::Options() const { return options_.base; }
bool UkfEstimator::IsInitialized() const { std::unique_lock lk(mutex_); return initialised_; }

std::optional<core::NavigationState> UkfEstimator::LatestState() const {
  std::unique_lock lk(mutex_);
  if (!initialised_) return std::nullopt;
  return BuildNavigationState();
}

void UkfEstimator::Reset() {
  std::unique_lock lk(mutex_);
  initialised_           = false;
  local_tangent_plane_   = std::nullopt;
  last_aiding_timestamp_ = std::nullopt;
  last_imu_timestamp_    = std::nullopt;
  imu_buffer_.Clear();
  state_ = UkfState{};
  FG_INFO("UKF | filter reset");
}

MeasurementUpdateReport UkfEstimator::AddMeasurement(const SensorMeasurement& measurement) {
  std::unique_lock lk(mutex_);

  // ── IMU: buffer ───────────────────────────────────────────────────────────
  if (const auto* imu = std::get_if<core::ImuMeasurement>(&measurement)) {
    const auto r = imu_buffer_.Insert(*imu);
    if (r == core::BufferInsertResult::RejectedOutOfOrder ||
        r == core::BufferInsertResult::RejectedNotComparable) {
      FG_WARN("UKF | IMU out-of-order, discarded");
      return {EstimatorUpdateResult::OutOfOrder};
    }
    return {EstimatorUpdateResult::Buffered};
  }

  // ── Initialise from first GNSS ────────────────────────────────────────────
  if (!initialised_) {
    if (const auto* gnss = std::get_if<core::GnssSolution>(&measurement)) {
      const bool ok = TryInitialiseFromGnss(*gnss);
      return {ok ? EstimatorUpdateResult::Accepted : EstimatorUpdateResult::Rejected,
              "UkfGnssLooselyCoupled"};
    }
    FG_DEBUG("UKF | measurement received before initialization, discarded");
    return {EstimatorUpdateResult::NotInitialized};
  }

  // ── Propagate IMU to measurement time ─────────────────────────────────────
  PropagateTo(estimation::GetTimestamp(measurement));

  // ── Dispatch to measurement models ────────────────────────────────────────
  const UkfUpdateContext ctx{local_tangent_plane_ ? &*local_tangent_plane_ : nullptr};

  for (auto& model : measurement_models_) {
    if (!model->CanHandle(measurement)) continue;

    auto report = ApplyMeasurementModel(*model, measurement, ctx);
    if (report.result == EstimatorUpdateResult::Accepted) {
      last_aiding_timestamp_ = estimation::GetTimestamp(measurement);
      if (report.correction_norm) {
        FG_DEBUG("UKF | {} accepted, correction={:.4f}m", report.model_name, *report.correction_norm);
        if (*report.correction_norm > 10.0) {
          FG_WARN("UKF | {} large correction={:.4f}m — possible sensor outlier",
                  report.model_name, *report.correction_norm);
        }
      }
    } else {
      FG_DEBUG("UKF | {} rejected (gate or invalid)", report.model_name);
    }
    return report;
  }
  FG_WARN("UKF | no measurement model matched for incoming measurement");
  return {EstimatorUpdateResult::Rejected};
}

EstimatorUpdateResult UkfEstimator::ProcessUntil(const core::Timestamp& timestamp) {
  std::unique_lock lk(mutex_);
  if (!initialised_) return EstimatorUpdateResult::NotInitialized;
  PropagateTo(timestamp);
  return EstimatorUpdateResult::Accepted;
}

// ── Sigma point helpers ───────────────────────────────────────────────────────

// Unpack error-state sigma point into a full NominalState
static NominalState UnpackSigmaPoint(const NominalState& mean, const StateVector& sigma) {
  NominalState s;
  s.position_enu_m        = core::Vec3<core::EnuFrame>(mean.position_enu_m.eigen()   + sigma.segment<3>(0));
  s.velocity_enu_mps      = core::Vec3<core::EnuFrame>(mean.velocity_enu_mps.eigen() + sigma.segment<3>(3));
  s.orientation_body_to_enu = (mean.orientation_body_to_enu * core::ExpMapSo3(sigma.segment<3>(6))).normalized();
  s.accel_bias_mps2       = core::Vec3<core::ImuFrame>(mean.accel_bias_mps2.eigen()  + sigma.segment<3>(9));
  s.gyro_bias_radps       = core::Vec3<core::ImuFrame>(mean.gyro_bias_radps.eigen()  + sigma.segment<3>(12));
  return s;
}

// Pack propagated state back to error-state relative to (updated) mean
static StateVector PackSigmaPoint(const NominalState& mean, const NominalState& s) {
  StateVector v;
  v.segment<3>(0)  = s.position_enu_m.eigen()   - mean.position_enu_m.eigen();
  v.segment<3>(3)  = s.velocity_enu_mps.eigen()  - mean.velocity_enu_mps.eigen();
  v.segment<3>(6)  = core::LogMapSo3(mean.orientation_body_to_enu.conjugate() * s.orientation_body_to_enu);
  v.segment<3>(9)  = s.accel_bias_mps2.eigen()   - mean.accel_bias_mps2.eigen();
  v.segment<3>(12) = s.gyro_bias_radps.eigen()   - mean.gyro_bias_radps.eigen();
  return v;
}

// Propagate one sigma point through the INS strapdown equations
static NominalState PropagateOnePoint(
    const NominalState& s,
    const core::ImuMeasurement& imu,
    double dt_s) {

  using Eigen::Vector3d;
  const Vector3d f     = imu.specific_force_mps2.eigen() - s.accel_bias_mps2.eigen();
  const Vector3d omega = imu.angular_rate_radps.eigen()  - s.gyro_bias_radps.eigen();
  const Eigen::Matrix3d R = s.orientation_body_to_enu.toRotationMatrix();
  const Vector3d g_enu(0.0, 0.0, -kGravityMps2);
  const Vector3d a_enu = R * f + g_enu;

  NominalState out;
  out.position_enu_m  = core::Vec3<core::EnuFrame>(
      s.position_enu_m.eigen() + s.velocity_enu_mps.eigen() * dt_s + 0.5 * a_enu * dt_s * dt_s);
  out.velocity_enu_mps = core::Vec3<core::EnuFrame>(s.velocity_enu_mps.eigen() + a_enu * dt_s);
  out.orientation_body_to_enu = (s.orientation_body_to_enu * core::ExpMapSo3(omega * dt_s)).normalized();
  out.accel_bias_mps2 = s.accel_bias_mps2;
  out.gyro_bias_radps = s.gyro_bias_radps;
  return out;
}

// ── GenerateSigmaPoints ───────────────────────────────────────────────────────
//
// X₀ = 0  (error state mean is always zero between updates)
// Xᵢ = +√((n+λ)P)ᵢ    i = 1..n
// Xₙ₊ᵢ = −√((n+λ)P)ᵢ  i = 1..n
//
void UkfEstimator::GenerateSigmaPoints() {
  const double n      = kStateSize;
  const double lambda = options_.sigma_params.alpha * options_.sigma_params.alpha *
                        (n + options_.sigma_params.kappa) - n;

  auto llt = ((n + lambda) * state_.covariance).llt();
  if (llt.info() != Eigen::Success) {
    FG_WARN("UKF | Cholesky failed, regularizing covariance (+1e-9·I)");
    state_.covariance += StateCovariance::Identity() * 1e-9;
    llt = ((n + lambda) * state_.covariance).llt();
  }
  const StateCovariance L = llt.matrixL();

  state_.sigma_points.col(0) = StateVector::Zero();
  for (int i = 0; i < kStateSize; ++i) {
    state_.sigma_points.col(i + 1)             =  L.col(i);
    state_.sigma_points.col(i + 1 + kStateSize) = -L.col(i);
  }
}

// ── PropagateImu ──────────────────────────────────────────────────────────────
//
// Unscented Transform applied to the INS strapdown equations.
// Steps:
//   1. Generate sigma points from current P.
//   2. Propagate each through f(x, imu, dt).
//   3. Recover new mean (translational: weighted sum; attitude: quaternion mean).
//   4. Recover new covariance + add process noise Q.
//
void UkfEstimator::PropagateImu(const core::ImuMeasurement& imu, double dt_s) {
  GenerateSigmaPoints();

  // Propagate all sigma points
  std::array<NominalState, kNumSigmaPoints> propagated;
  for (int i = 0; i < kNumSigmaPoints; ++i) {
    const NominalState s = UnpackSigmaPoint(state_.nominal, state_.sigma_points.col(i));
    propagated[i] = PropagateOnePoint(s, imu, dt_s);
  }

  // ── Recover translational mean ────────────────────────────────────────────
  NominalState new_nom;
  Eigen::Vector3d mean_p  = Eigen::Vector3d::Zero();
  Eigen::Vector3d mean_v  = Eigen::Vector3d::Zero();
  Eigen::Vector3d mean_ba = Eigen::Vector3d::Zero();
  Eigen::Vector3d mean_bg = Eigen::Vector3d::Zero();
  for (int i = 0; i < kNumSigmaPoints; ++i) {
    mean_p  += weights_.mean_weights(i) * propagated[i].position_enu_m.eigen();
    mean_v  += weights_.mean_weights(i) * propagated[i].velocity_enu_mps.eigen();
    mean_ba += weights_.mean_weights(i) * propagated[i].accel_bias_mps2.eigen();
    mean_bg += weights_.mean_weights(i) * propagated[i].gyro_bias_radps.eigen();
  }
  new_nom.position_enu_m  = core::Vec3<core::EnuFrame>(mean_p);
  new_nom.velocity_enu_mps = core::Vec3<core::EnuFrame>(mean_v);
  new_nom.accel_bias_mps2 = core::Vec3<core::ImuFrame>(mean_ba);
  new_nom.gyro_bias_radps = core::Vec3<core::ImuFrame>(mean_bg);
  new_nom.timestamp        = imu.timestamp;

  // ── Recover attitude mean (iterative quaternion averaging) ────────────────
  //
  // Start from sigma point 0 (centre point = current mean propagated).
  // Iterate: compute weighted mean of rotation vectors relative to q_mean,
  // then fold into q_mean.  Converges in 1–2 steps for small spreads.
  //
  Eigen::Quaterniond q_mean = propagated[0].orientation_body_to_enu;
  for (int iter = 0; iter < 5; ++iter) {
    Eigen::Vector3d mean_dtheta = Eigen::Vector3d::Zero();
    for (int i = 0; i < kNumSigmaPoints; ++i) {
      mean_dtheta += weights_.mean_weights(i) *
                     core::LogMapSo3(q_mean.conjugate() * propagated[i].orientation_body_to_enu);
    }
    q_mean = (q_mean * core::ExpMapSo3(mean_dtheta)).normalized();
    if (mean_dtheta.norm() < 1e-10) break;
  }
  new_nom.orientation_body_to_enu = q_mean;

  // ── Recover new covariance ────────────────────────────────────────────────
  StateCovariance new_cov = StateCovariance::Zero();
  for (int i = 0; i < kNumSigmaPoints; ++i) {
    const StateVector dx = PackSigmaPoint(new_nom, propagated[i]);
    new_cov += weights_.cov_weights(i) * dx * dx.transpose();
  }

  // ── Add process noise Q = G Qc Gᵀ dt ─────────────────────────────────────
  new_cov += BuildProcessNoise(imu, dt_s);

  state_.nominal    = new_nom;
  state_.covariance = core::SymmetrizeCovariance(new_cov);
}

void UkfEstimator::PropagateTo(const core::Timestamp& target) {
  const auto batch = imu_buffer_.PopUntil(target);
  for (const auto& imu : batch) {
    if (!state_.nominal.timestamp.has_steady && !state_.nominal.timestamp.has_gps) {
      state_.nominal.timestamp = imu.timestamp;
      continue;
    }
    const auto dt_opt = core::TimeDifference(imu.timestamp, state_.nominal.timestamp);
    if (!dt_opt) continue;
    const double dt_s = dt_opt->seconds();
    if (dt_s < options_.min_imu_dt_s) continue;

    double remaining = dt_s;
    while (remaining > 0.0) {
      const double step = std::min(remaining, options_.max_imu_dt_s);
      PropagateImu(imu, step);
      remaining -= step;
    }
    state_.nominal.timestamp = imu.timestamp;
  }
}

// ── BuildProcessNoise ─────────────────────────────────────────────────────────
//
// Q = G Qc Gᵀ dt   (same formula as EKF, same structure)
//
StateCovariance UkfEstimator::BuildProcessNoise(
    const core::ImuMeasurement& imu, double dt_s) const {

  const Eigen::Matrix3d R = state_.nominal.orientation_body_to_enu.toRotationMatrix();
  const Eigen::Vector3d f = imu.specific_force_mps2.eigen() - state_.nominal.accel_bias_mps2.eigen();

  // G (15×12)
  Eigen::Matrix<double, kStateSize, 12> G = Eigen::Matrix<double, kStateSize, 12>::Zero();
  G.block<3, 3>(3, 0)  = -R;                         // accel noise  → δv̇
  G.block<3, 3>(6, 3)  = -Eigen::Matrix3d::Identity(); // gyro noise   → δθ̇
  G.block<3, 3>(9, 6)  =  Eigen::Matrix3d::Identity(); // accel RW     → δḃa
  G.block<3, 3>(12, 9) =  Eigen::Matrix3d::Identity(); // gyro RW      → δḃg
  (void)f;  // f not used in Q (only needed for EKF F matrix cross-term)

  const double qa  = options_.imu_noise.accel_noise_density_mps2_per_sqrthz;
  const double qg  = options_.imu_noise.gyro_noise_density_radps_per_sqrthz;
  const double qba = options_.imu_noise.accel_random_walk_mps3_per_sqrthz;
  const double qbg = options_.imu_noise.gyro_random_walk_radps2_per_sqrthz;

  Eigen::Matrix<double, 12, 12> Qc = Eigen::Matrix<double, 12, 12>::Zero();
  Qc.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (qa  * qa);
  Qc.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (qg  * qg);
  Qc.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * (qba * qba);
  Qc.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * (qbg * qbg);

  return (G * Qc * G.transpose() * dt_s).eval();
}

// ── ApplyMeasurementModel ─────────────────────────────────────────────────────
//
// Unscented transform for the measurement update.
//
//  1. Generate sigma points from current P.
//  2. Pass each through h(xᵢ) → Z_i  (model.Predict).
//  3. Weighted mean z̄ = Σ Wm_i Z_i.
//  4. Innovation covariance S = Σ Wc_i (Z_i−z̄)(Z_i−z̄)ᵀ + R.
//  5. Cross-covariance Pxz = Σ Wc_i σ_i (Z_i−z̄)ᵀ.
//  6. Kalman gain K = Pxz S⁻¹.
//  7. Innovation δz = z_obs − z̄.
//  8. State update: inject K δz into nominal.
//  9. Covariance update P ← P − K S Kᵀ.
//
MeasurementUpdateReport UkfEstimator::ApplyMeasurementModel(
    IUkfMeasurementModel& model,
    const SensorMeasurement& measurement,
    const UkfUpdateContext& ctx) {

  const auto z_obs_opt = model.Observe(measurement, ctx);
  if (!z_obs_opt) return {EstimatorUpdateResult::Rejected, model.Name()};
  const Eigen::VectorXd& z_obs = *z_obs_opt;

  const int m = static_cast<int>(z_obs.size());
  const Eigen::MatrixXd R_meas = model.NoiseCovariance(measurement, ctx);

  GenerateSigmaPoints();

  // ── Transform sigma points through h ──────────────────────────────────────
  Eigen::MatrixXd Z(m, kNumSigmaPoints);
  for (int i = 0; i < kNumSigmaPoints; ++i) {
    const NominalState s = UnpackSigmaPoint(state_.nominal, state_.sigma_points.col(i));
    Z.col(i) = model.Predict(s, ctx);
  }

  // ── Predicted measurement mean ────────────────────────────────────────────
  Eigen::VectorXd z_pred = Eigen::VectorXd::Zero(m);
  for (int i = 0; i < kNumSigmaPoints; ++i) {
    z_pred += weights_.mean_weights(i) * Z.col(i);
  }

  // ── Innovation covariance S and cross-covariance Pxz ─────────────────────
  Eigen::MatrixXd S   = R_meas;
  Eigen::MatrixXd Pxz = Eigen::MatrixXd::Zero(kStateSize, m);

  for (int i = 0; i < kNumSigmaPoints; ++i) {
    const Eigen::VectorXd dz = model.Innovation(Z.col(i), z_pred);
    S   += weights_.cov_weights(i) * dz * dz.transpose();
    Pxz += weights_.cov_weights(i) * state_.sigma_points.col(i) * dz.transpose();
  }

  // ── Kalman gain ───────────────────────────────────────────────────────────
  const Eigen::MatrixXd K = S.ldlt().solve(Pxz.transpose()).transpose();

  // ── State and covariance update ───────────────────────────────────────────
  const Eigen::VectorXd delta_x = K * model.Innovation(z_obs, z_pred);

  auto& nom = state_.nominal;
  nom.position_enu_m  = core::Vec3<core::EnuFrame>(nom.position_enu_m.eigen()   + delta_x.segment<3>(0));
  nom.velocity_enu_mps = core::Vec3<core::EnuFrame>(nom.velocity_enu_mps.eigen() + delta_x.segment<3>(3));
  nom.orientation_body_to_enu =
      (nom.orientation_body_to_enu * core::ExpMapSo3(delta_x.segment<3>(6))).normalized();
  nom.accel_bias_mps2 = core::Vec3<core::ImuFrame>(nom.accel_bias_mps2.eigen()  + delta_x.segment<3>(9));
  nom.gyro_bias_radps = core::Vec3<core::ImuFrame>(nom.gyro_bias_radps.eigen()  + delta_x.segment<3>(12));

  state_.covariance -= K * S * K.transpose();
  state_.covariance  = core::SymmetrizeCovariance(state_.covariance);

  return {EstimatorUpdateResult::Accepted, model.Name(), delta_x.norm()};
}

// ── TryInitialiseFromGnss ─────────────────────────────────────────────────────

bool UkfEstimator::TryInitialiseFromGnss(const core::GnssSolution& gnss) {
  if (gnss.fix_type == core::GnssFixType::NoFix) {
    FG_WARN("UKF | GNSS init failed: NoFix");
    return false;
  }
  if (gnss.validity != core::MeasurementValidity::Valid) {
    FG_WARN("UKF | GNSS init failed: measurement not valid");
    return false;
  }

  const core::Lla origin_lla = core::EcefToLla(gnss.position_ecef_m);
  local_tangent_plane_ = core::LocalTangentPlane(origin_lla);
  const Eigen::Matrix3d R = local_tangent_plane_->ecef_to_enu_rotation();

  auto& nom = state_.nominal;
  nom.timestamp              = gnss.timestamp;
  nom.position_enu_m         = core::Vec3<core::EnuFrame>(0.0, 0.0, 0.0);
  nom.velocity_enu_mps       = core::Vec3<core::EnuFrame>(R * gnss.velocity_ecef_mps.eigen());
  nom.orientation_body_to_enu = Eigen::Quaterniond::Identity();
  nom.accel_bias_mps2        = core::Vec3<core::ImuFrame>(0.0, 0.0, 0.0);
  nom.gyro_bias_radps        = core::Vec3<core::ImuFrame>(0.0, 0.0, 0.0);

  state_.covariance = StateCovariance::Zero();

  Eigen::Matrix3d cov_pos = R * gnss.position_covariance_ecef_m2 * R.transpose();
  Eigen::Matrix3d cov_vel = R * gnss.velocity_covariance_ecef_m2ps2 * R.transpose();
  if (cov_pos.norm() < 1e-12) {
    cov_pos = Eigen::Matrix3d::Identity() * (kInitPosSigmaM  * kInitPosSigmaM);
    cov_vel = Eigen::Matrix3d::Identity() * (kInitVelSigmaMps * kInitVelSigmaMps);
  }

  state_.covariance.block<3, 3>(0, 0)   = cov_pos;
  state_.covariance.block<3, 3>(3, 3)   = cov_vel;
  state_.covariance.block<3, 3>(6, 6)   = Eigen::Matrix3d::Identity() * (kInitAttSigmaRad * kInitAttSigmaRad);
  state_.covariance.block<3, 3>(9, 9)   = Eigen::Matrix3d::Identity() * (kInitAbaSigma    * kInitAbaSigma);
  state_.covariance.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * (kInitGbaSigma    * kInitGbaSigma);

  state_.sigma_points = SigmaMatrix::Zero();
  initialised_ = true;
  FG_INFO("UKF | initialized — LTP origin lat={:.6f}° lon={:.6f}° alt={:.1f}m",
          origin_lla.latitude_rad * (180.0 / M_PI),
          origin_lla.longitude_rad * (180.0 / M_PI),
          origin_lla.altitude_m);
  return true;
}

// ── BuildNavigationState ──────────────────────────────────────────────────────

core::NavigationState UkfEstimator::BuildNavigationState() const {
  using namespace core;
  const auto& nom = state_.nominal;

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

  out.covariance = state_.covariance;

  out.quality.initialized        = initialised_;
  out.quality.position_accuracy_m  = std::sqrt(state_.covariance.block<3,3>(0,0).trace() / 3.0);
  out.quality.velocity_accuracy_mps = std::sqrt(state_.covariance.block<3,3>(3,3).trace() / 3.0);
  out.quality.attitude_accuracy_rad = std::sqrt(state_.covariance.block<3,3>(6,6).trace() / 3.0);

  if (last_aiding_timestamp_ && nom.timestamp.has_steady && last_aiding_timestamp_->has_steady) {
    const auto age = TimeDifference(nom.timestamp, *last_aiding_timestamp_);
    out.status = (age && age->seconds() > options_.dead_reckoning_threshold_s)
                     ? NavigationStatus::DeadReckoning
                     : NavigationStatus::Nominal;
  } else {
    out.status = NavigationStatus::DeadReckoning;
  }

  out.mode = measurement_models_.empty() ? EstimatorMode::InertialOnly
                                         : EstimatorMode::MultiSensorFusion;
  out.sensors.imu.health           = SensorHealth::Healthy;
  out.sensors.imu.used_in_solution = true;
  return out;
}

}  // namespace falconguide::estimation::ukf
