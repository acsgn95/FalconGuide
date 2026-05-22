#include "falconguide/estimation/backends/gtsam/gtsam_estimator.hpp"
#include "falconguide/core/coordinates.hpp"
#include "falconguide/logger/logger.hpp"

#ifdef FALCONGUIDE_HAVE_GTSAM

#include "falconguide/estimation/backends/ceres/imu_preintegrator.hpp"

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

#include <mutex>

namespace falconguide::estimation::gtsam_backend {

using gtsam::symbol_shorthand::X;  // pose
using gtsam::symbol_shorthand::V;  // velocity
using gtsam::symbol_shorthand::B;  // bias

static constexpr double kGravityMps2 = 9.80665;

// ── Helper conversions ────────────────────────────────────────────────────────

static gtsam::Pose3 KfToPose3(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
  return gtsam::Pose3(gtsam::Rot3(q), gtsam::Point3(p));
}

static gtsam::imuBias::ConstantBias ToGtsamBias(const Eigen::Vector3d& ba,
                                                  const Eigen::Vector3d& bg) {
  return {ba, bg};
}

// ── Implementation ────────────────────────────────────────────────────────────

class GtsamFactorGraphEstimator::Impl {
 public:
  explicit Impl(GtsamOptions opts) : opts_(std::move(opts)) {
    // Build iSAM2 params
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = opts_.isam2_policy.relinearise_threshold_pose;
    params.relinearizeSkip      = opts_.isam2_policy.relinearise_skip;
    if (!opts_.isam2_policy.use_cholesky)
      params.factorization = gtsam::ISAM2Params::QR;
    isam_ = std::make_unique<gtsam::ISAM2>(params);

    // Build IMU preintegration params (GTSAM style)
    auto preint_params = gtsam::PreintegrationCombinedParams::MakeSharedU(kGravityMps2);
    const double na  = 3e-3, ng  = 1.5e-4;
    const double nba = 3e-5, nbg = 2e-6;
    preint_params->accelerometerCovariance  = Eigen::Matrix3d::Identity() * na  * na;
    preint_params->gyroscopeCovariance      = Eigen::Matrix3d::Identity() * ng  * ng;
    preint_params->biasAccCovariance        = Eigen::Matrix3d::Identity() * nba * nba;
    preint_params->biasOmegaCovariance      = Eigen::Matrix3d::Identity() * nbg * nbg;
    preint_params->integrationCovariance    = Eigen::Matrix3d::Identity() * 1e-8;
    preint_params->biasAccOmegaInt          = Eigen::Matrix<double,6,6>::Identity() * 1e-5;
    preint_params_ = preint_params;
  }

  GtsamOptions opts_;
  mutable std::mutex mutex_;

  std::unique_ptr<gtsam::ISAM2> isam_;
  boost::shared_ptr<gtsam::PreintegrationCombinedParams> preint_params_;
  std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> preint_;

  gtsam::NonlinearFactorGraph new_factors_;
  gtsam::Values               new_values_;

  uint64_t next_idx_{0};
  bool     initialised_{false};

  core::LocalTangentPlane ltp_;
  core::NavigationState   latest_state_;

  // Current best estimate (maintained for state output)
  gtsam::Values current_estimate_;

  // ── IMU ───────────────────────────────────────────────────────────────────

  MeasurementUpdateReport AddImu(const core::ImuMeasurement& imu) {
    if (!initialised_) {
      imu_buf_.push_back(imu);
      if (imu_buf_.size() > 2000) imu_buf_.pop_front();
      return {EstimatorUpdateResult::Buffered};
    }
    if (preint_) {
      preint_->integrateMeasurement(
          imu.specific_force_mps2.eigen(),
          imu.angular_rate_radps.eigen(),
          prev_imu_ts_.has_steady
              ? (imu.timestamp.steady - prev_imu_ts_.steady).seconds()
              : 0.01);
    }
    prev_imu_ts_ = imu.timestamp;
    return {EstimatorUpdateResult::Buffered};
  }

  MeasurementUpdateReport AddGnss(const core::GnssSolution& gnss) {
    if (gnss.validity != core::MeasurementValidity::Valid)
      return {EstimatorUpdateResult::Rejected};
    if (!initialised_) return InitFromGnss(gnss);
    return NewNode(gnss);
  }

  MeasurementUpdateReport InitFromGnss(const core::GnssSolution& gnss) {
    const core::Lla lla = core::EcefToLla(gnss.position_ecef_m);
    ltp_ = core::LocalTangentPlane(lla);

    // Estimate attitude from IMU gravity
    Eigen::Vector3d grav_body(0, 0, -kGravityMps2);
    if (!imu_buf_.empty()) {
      Eigen::Vector3d sum = Eigen::Vector3d::Zero();
      for (const auto& i : imu_buf_) sum += i.specific_force_mps2.eigen();
      grav_body = -(sum / static_cast<double>(imu_buf_.size()));
    }
    const Eigen::Vector3d grav_enu(0, 0, -kGravityMps2);
    const Eigen::Quaterniond q_init =
        Eigen::Quaterniond::FromTwoVectors(-grav_body.normalized(), grav_enu.normalized());

    const gtsam::Pose3   init_pose = KfToPose3(Eigen::Vector3d::Zero(), q_init);
    const gtsam::Vector3 init_vel  = gtsam::Vector3::Zero();
    const gtsam::imuBias::ConstantBias init_bias;

    const uint64_t idx = next_idx_++;

    // Prior noise models
    auto prior_noise_pose = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector6() << 0.01, 0.01, 0.01, 10.0, 10.0, 10.0).finished());
    auto prior_noise_vel  = gtsam::noiseModel::Isotropic::Sigma(3, 1.0);
    auto prior_noise_bias = gtsam::noiseModel::Isotropic::Sigma(6, 0.01);

    new_factors_.addPrior(X(idx), init_pose, prior_noise_pose);
    new_factors_.addPrior(V(idx), init_vel,  prior_noise_vel);
    new_factors_.addPrior(B(idx), init_bias, prior_noise_bias);

    new_values_.insert(X(idx), init_pose);
    new_values_.insert(V(idx), init_vel);
    new_values_.insert(B(idx), init_bias);

    // GNSS position factor
    const Eigen::Vector3d pos_enu = Eigen::Vector3d::Zero();  // at origin
    auto gnss_noise = gtsam::noiseModel::Gaussian::Covariance(
        ltp_.ecef_to_enu_rotation() *
        gnss.position_covariance_ecef_m2 *
        ltp_.ecef_to_enu_rotation().transpose());
    new_factors_.emplace_shared<gtsam::GPSFactor>(X(idx),
        gtsam::Point3(pos_enu), gnss_noise);

    isam_->update(new_factors_, new_values_);
    current_estimate_ = isam_->calculateEstimate();
    new_factors_.resize(0);
    new_values_.clear();

    // Start preintegration
    prev_bias_ = init_bias;
    preint_    = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(
        preint_params_, prev_bias_);
    prev_pose_idx_ = idx;
    prev_imu_ts_ = gnss.timestamp;
    initialised_ = true;

    BuildNavigationState(idx);
    FG_INFO("GTSAM | initialised at GNSS fix");
    return {EstimatorUpdateResult::Accepted, "GtsamFactorGraph"};
  }

  MeasurementUpdateReport NewNode(const core::GnssSolution& gnss) {
    const Eigen::Matrix3d R_enu = ltp_.ecef_to_enu_rotation();
    const Eigen::Vector3d pos_enu =
        R_enu * (gnss.position_ecef_m.eigen() - ltp_.origin_ecef().eigen());

    const uint64_t prev_idx = prev_pose_idx_;
    const uint64_t cur_idx  = next_idx_++;

    // Propagate using preintegrated IMU for initial guess
    gtsam::NavState prev_nav(current_estimate_.at<gtsam::Pose3>(X(prev_idx)),
                              current_estimate_.at<gtsam::Vector3>(V(prev_idx)));
    gtsam::NavState cur_nav = preint_->predict(prev_nav, prev_bias_);

    new_values_.insert(X(cur_idx), cur_nav.pose());
    new_values_.insert(V(cur_idx), cur_nav.velocity());
    new_values_.insert(B(cur_idx), prev_bias_);

    // Combined IMU factor
    new_factors_.emplace_shared<gtsam::CombinedImuFactor>(
        X(prev_idx), V(prev_idx), X(cur_idx), V(cur_idx),
        B(prev_idx), B(cur_idx), *preint_);

    // GNSS position factor
    const Eigen::Matrix3d cov_enu = R_enu * gnss.position_covariance_ecef_m2 * R_enu.transpose();
    auto gnss_noise = gtsam::noiseModel::Gaussian::Covariance(cov_enu);
    new_factors_.emplace_shared<gtsam::GPSFactor>(X(cur_idx),
        gtsam::Point3(pos_enu), gnss_noise);

    // GNSS velocity factor (via velocity prior)
    const Eigen::Vector3d vel_enu = R_enu * gnss.velocity_ecef_mps.eigen();
    const Eigen::Matrix3d vcov    = R_enu * gnss.velocity_covariance_ecef_m2ps2 * R_enu.transpose();
    auto vel_noise = gtsam::noiseModel::Gaussian::Covariance(vcov);
    new_factors_.addPrior(V(cur_idx), gtsam::Vector3(vel_enu), vel_noise);

    isam_->update(new_factors_, new_values_);
    current_estimate_ = isam_->calculateEstimate();
    new_factors_.resize(0);
    new_values_.clear();

    // Update bias and restart preintegrator
    prev_bias_     = current_estimate_.at<gtsam::imuBias::ConstantBias>(B(cur_idx));
    preint_        = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(
        preint_params_, prev_bias_);
    prev_pose_idx_ = cur_idx;
    prev_imu_ts_   = gnss.timestamp;

    BuildNavigationState(cur_idx);
    return {EstimatorUpdateResult::Accepted, "GtsamFactorGraph"};
  }

  void BuildNavigationState(uint64_t idx) {
    const auto pose = current_estimate_.at<gtsam::Pose3>(X(idx));
    const auto vel  = current_estimate_.at<gtsam::Vector3>(V(idx));
    const auto bias = current_estimate_.at<gtsam::imuBias::ConstantBias>(B(idx));

    const Eigen::Vector3d p_enu = pose.translation();
    const Eigen::Quaterniond q  = Eigen::Quaterniond(pose.rotation().matrix());
    const Eigen::Vector3d v_enu(vel(0), vel(1), vel(2));

    latest_state_.position_enu_m   = core::Vec3<core::EnuFrame>(p_enu.x(), p_enu.y(), p_enu.z());
    latest_state_.velocity_enu_mps = core::Vec3<core::EnuFrame>(v_enu.x(), v_enu.y(), v_enu.z());
    latest_state_.orientation_body_to_enu = q;

    const Eigen::Matrix3d R_enu_to_ecef = ltp_.ecef_to_enu_rotation().transpose();
    const Eigen::Vector3d p_ecef = ltp_.origin_ecef().eigen() + R_enu_to_ecef * p_enu;
    latest_state_.position_ecef_m   = core::Vec3<core::EcefFrame>(p_ecef.x(), p_ecef.y(), p_ecef.z());
    const Eigen::Vector3d v_ecef    = R_enu_to_ecef * v_enu;
    latest_state_.velocity_ecef_mps = core::Vec3<core::EcefFrame>(v_ecef.x(), v_ecef.y(), v_ecef.z());

    latest_state_.status = core::NavigationStatus::Nominal;
    latest_state_.quality.initialized = true;
    (void)bias;  // could report as sensor health
  }

  std::deque<core::ImuMeasurement> imu_buf_;
  gtsam::imuBias::ConstantBias prev_bias_;
  uint64_t prev_pose_idx_{0};
  core::Timestamp prev_imu_ts_;
};

// ── Public API ────────────────────────────────────────────────────────────────

GtsamFactorGraphEstimator::GtsamFactorGraphEstimator(GtsamOptions options)
    : options_(std::move(options)),
      impl_(std::make_unique<Impl>(options_)),
      graph_(impl_->graph_) {}

GtsamFactorGraphEstimator::~GtsamFactorGraphEstimator() = default;

EstimatorInfo GtsamFactorGraphEstimator::Info() const {
  return {EstimatorBackend::GtsamFactorGraph, "GtsamFactorGraph", "1.0"};
}

const EstimatorOptions& GtsamFactorGraphEstimator::Options() const {
  return options_.base;
}

bool GtsamFactorGraphEstimator::IsInitialized() const {
  std::unique_lock lk(impl_->mutex_);
  return impl_->initialised_;
}

std::optional<core::NavigationState> GtsamFactorGraphEstimator::LatestState() const {
  std::unique_lock lk(impl_->mutex_);
  if (!impl_->initialised_) return std::nullopt;
  return impl_->latest_state_;
}

MeasurementUpdateReport GtsamFactorGraphEstimator::AddMeasurement(
    const SensorMeasurement& measurement) {
  std::unique_lock lk(impl_->mutex_);
  if (const auto* imu = std::get_if<core::ImuMeasurement>(&measurement))
    return impl_->AddImu(*imu);
  if (const auto* gnss = std::get_if<core::GnssSolution>(&measurement))
    return impl_->AddGnss(*gnss);
  return {EstimatorUpdateResult::Rejected};
}

EstimatorUpdateResult GtsamFactorGraphEstimator::ProcessUntil(
    const core::Timestamp& /*ts*/) {
  return EstimatorUpdateResult::Accepted;
}

void GtsamFactorGraphEstimator::Reset() {
  std::unique_lock lk(impl_->mutex_);
  impl_ = std::make_unique<Impl>(options_);
  FG_INFO("GTSAM | reset");
}

const FactorGraphState& GtsamFactorGraphEstimator::GraphState() const {
  return impl_->graph_;
}

}  // namespace falconguide::estimation::gtsam_backend

#else  // FALCONGUIDE_HAVE_GTSAM not defined

#include <stdexcept>

namespace falconguide::estimation::gtsam_backend {

GtsamFactorGraphEstimator::GtsamFactorGraphEstimator(GtsamOptions) {
  throw std::runtime_error("GtsamFactorGraphEstimator: built without GTSAM support. "
                           "Enable FALCONGUIDE_ENABLE_GTSAM in CMake.");
}
EstimatorInfo GtsamFactorGraphEstimator::Info() const { return {}; }
const EstimatorOptions& GtsamFactorGraphEstimator::Options() const {
  static EstimatorOptions dummy; return dummy;
}
bool GtsamFactorGraphEstimator::IsInitialized() const { return false; }
std::optional<core::NavigationState> GtsamFactorGraphEstimator::LatestState() const { return {}; }
MeasurementUpdateReport GtsamFactorGraphEstimator::AddMeasurement(const SensorMeasurement&) { return {}; }
EstimatorUpdateResult GtsamFactorGraphEstimator::ProcessUntil(const core::Timestamp&) { return EstimatorUpdateResult::BackendError; }
void GtsamFactorGraphEstimator::Reset() {}
const FactorGraphState& GtsamFactorGraphEstimator::GraphState() const {
  static FactorGraphState dummy; return dummy;
}

}  // namespace falconguide::estimation::gtsam_backend

#endif  // FALCONGUIDE_HAVE_GTSAM
