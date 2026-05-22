#include "falconguide/estimation/backends/ceres/ceres_estimator.hpp"

#ifdef FALCONGUIDE_HAVE_CERES

#include "falconguide/estimation/backends/ceres/ceres_cost_functions.hpp"
#include "falconguide/estimation/backends/ceres/imu_preintegrator.hpp"
#include "falconguide/estimation/measurement_variant_helpers.hpp"
#include "falconguide/core/coordinates.hpp"
#include "falconguide/core/math.hpp"
#include "falconguide/core/time_helpers.hpp"
#include "falconguide/logger/logger.hpp"

#include <ceres/ceres.h>
#include <ceres/version.h>

#include <deque>
#include <mutex>
#include <optional>

// Ceres 2.1+ replaced LocalParameterization with Manifold API.
#if CERES_VERSION_MAJOR > 2 || (CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 1)
#  define FG_CERES_SET_QUAT_MANIFOLD(problem, ptr) \
     (problem).SetManifold((ptr), new ceres::EigenQuaternionManifold())
#else
#  define FG_CERES_SET_QUAT_MANIFOLD(problem, ptr) \
     (problem).SetParameterization((ptr), new ceres::EigenQuaternionParameterization())
#endif

namespace falconguide::estimation::ceres_backend {

// ── Parameter block helpers ───────────────────────────────────────────────────

// pose_block[7] = [px, py, pz, qx, qy, qz, qw]
struct PoseBlock {
    double data[7]{0, 0, 0, 0, 0, 0, 1};

    void Set(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
        data[0] = p.x();
        data[1] = p.y();
        data[2] = p.z();
        data[3] = q.x();
        data[4] = q.y();
        data[5] = q.z();
        data[6] = q.w();
    }
    Eigen::Vector3d Position() const { return {data[0], data[1], data[2]}; }
    Eigen::Quaterniond Rotation() const { return Eigen::Quaterniond(data[6], data[3], data[4], data[5]).normalized(); }
};

// vel_block[3]  = [vx, vy, vz]
// bias_block[6] = [bax, bay, baz, bgx, bgy, bgz]

struct WindowKeyframe {
    PoseBlock pose;
    double vel[3]{0, 0, 0};
    double bias[6]{0, 0, 0, 0, 0, 0};  // [accel(3), gyro(3)]
    core::Timestamp timestamp;

    Eigen::Vector3d P() const { return pose.Position(); }
    Eigen::Quaterniond Q() const { return pose.Rotation(); }
    Eigen::Vector3d V() const { return {vel[0], vel[1], vel[2]}; }
    Eigen::Vector3d Ba() const { return {bias[0], bias[1], bias[2]}; }
    Eigen::Vector3d Bg() const { return {bias[3], bias[4], bias[5]}; }

    void SetV(const Eigen::Vector3d& v) {
        vel[0] = v.x();
        vel[1] = v.y();
        vel[2] = v.z();
    }
    void SetBias(const Eigen::Vector3d& ba, const Eigen::Vector3d& bg) {
        bias[0] = ba.x();
        bias[1] = ba.y();
        bias[2] = ba.z();
        bias[3] = bg.x();
        bias[4] = bg.y();
        bias[5] = bg.z();
    }
};

// ── Implementation ────────────────────────────────────────────────────────────

class CeresSlidingWindowEstimator::Impl {
   public:
    explicit Impl(CeresOptions opts) : opts_(std::move(opts)) {}

    CeresOptions opts_;
    mutable std::mutex mutex_;

    // Sliding window
    std::deque<WindowKeyframe> window_;
    std::deque<ImuPreintegration> imu_factors_;  // factor between window_[i] and window_[i+1]

    // IMU integration between the last keyframe and now
    ImuPreintegrator preint_;
    bool preint_active_{false};

    // IMU sample buffer (for late measurements, used before first KF)
    std::deque<core::ImuMeasurement> imu_buffer_;
    static constexpr std::size_t kMaxImuBuffer = 2000;

    bool initialised_{false};
    std::optional<core::LocalTangentPlane> ltp_;  // ENU reference frame
    core::Timestamp last_kf_time_;

    // Latest state (from last optimisation)
    core::NavigationState latest_state_;

    // Public diagnostics snapshot rebuilt on demand from the solver-owned
    // parameter blocks.
    mutable SlidingWindowState diagnostic_state_;

    // ── Init ──────────────────────────────────────────────────────────────────

    MeasurementUpdateReport AddImu(const core::ImuMeasurement& imu) {
        if (imu_buffer_.size() < kMaxImuBuffer) imu_buffer_.push_back(imu);
        if (preint_active_) preint_.Integrate(imu);
        return {EstimatorUpdateResult::Buffered};
    }

    MeasurementUpdateReport AddGnss(const core::GnssSolution& gnss) {
        if (gnss.validity != core::MeasurementValidity::Valid) return {EstimatorUpdateResult::Rejected};

        if (!initialised_) return InitFromGnss(gnss);

        // Check distance/time threshold for new keyframe
        const Eigen::Vector3d pos_enu =
            ltp_->ecef_to_enu_rotation() * (gnss.position_ecef_m.eigen() - ltp_->origin_ecef_m().eigen());

        const Eigen::Vector3d cur_pos = window_.back().P();
        const double dist = (pos_enu - cur_pos).norm();
        const double dt = (gnss.timestamp.steady - last_kf_time_.steady).seconds();
        if (dist < opts_.keyframe_min_distance_m && dt < 0.2) {
            // Update latest keyframe GNSS estimate without creating a new KF
            return UpdateLatestGnss(gnss, pos_enu);
        }

        return CreateKeyframe(gnss, pos_enu);
    }

    MeasurementUpdateReport InitFromGnss(const core::GnssSolution& gnss) {
        // Establish ENU reference frame
        const core::Lla lla = core::EcefToLla(gnss.position_ecef_m);
        ltp_.emplace(lla);

        // Create first keyframe at origin
        WindowKeyframe kf;
        kf.timestamp = gnss.timestamp;
        kf.pose.Set(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
        kf.SetV(Eigen::Vector3d::Zero());
        // Estimate gravity direction from buffered IMU
        Eigen::Vector3d gravity_body = Eigen::Vector3d(0, 0, -kGravityMps2);
        if (!imu_buffer_.empty()) {
            Eigen::Vector3d sum = Eigen::Vector3d::Zero();
            for (const auto& i : imu_buffer_) sum += i.specific_force_mps2.eigen();
            gravity_body = -(sum / static_cast<double>(imu_buffer_.size()));
        }
        // Align body -Z with gravity (level alignment)
        const Eigen::Vector3d grav_enu(0.0, 0.0, -kGravityMps2);
        const Eigen::Quaterniond q_init =
            Eigen::Quaterniond::FromTwoVectors(-gravity_body.normalized(), grav_enu.normalized());
        kf.pose.Set(Eigen::Vector3d::Zero(), q_init);
        kf.SetBias(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

        window_.push_back(kf);
        last_kf_time_ = gnss.timestamp;
        initialised_ = true;

        // Start preintegrator for next keyframe
        preint_.Reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), gnss.timestamp);
        // Re-play buffered IMU into the preintegrator
        for (const auto& i : imu_buffer_) {
            if (!core::IsBefore(i.timestamp, gnss.timestamp)) preint_.Integrate(i);
        }
        preint_active_ = true;

        BuildNavigationState(window_.back());
        FG_INFO("Ceres | initialised at GNSS fix");
        return {EstimatorUpdateResult::Accepted, "CeresSlidingWindow"};
    }

    MeasurementUpdateReport UpdateLatestGnss(const core::GnssSolution& gnss, const Eigen::Vector3d& pos_enu) {
        // Just run a quick single-KF optimisation to refine position/velocity
        WindowKeyframe& kf = window_.back();
        ceres::Problem problem;

        // GNSS factors on latest keyframe
        const auto cov_pos = gnss.position_covariance_ecef_m2.eval();
        const Eigen::Matrix3d R_enu = ltp_->ecef_to_enu_rotation();
        const Eigen::Matrix3d cov_enu = R_enu * cov_pos * R_enu.transpose();

        problem.AddResidualBlock(GnssPositionCost::Create(pos_enu, cov_enu), nullptr, kf.pose.data);
        problem.AddResidualBlock(
            GnssVelocityCost::Create(R_enu * gnss.velocity_ecef_mps.eigen(),
                                     R_enu * gnss.velocity_covariance_ecef_m2ps2 * R_enu.transpose()),
            nullptr, kf.vel);

        // Quaternion parametrisation
        FG_CERES_SET_QUAT_MANIFOLD(problem, kf.pose.data);
        problem.SetParameterBlockConstant(&kf.pose.data[0]);  // fix position block
        problem.SetParameterBlockVariable(kf.vel);

        ceres::Solver::Options sopts;
        sopts.max_num_iterations = 5;
        sopts.logging_type = ceres::SILENT;
        ceres::Solver::Summary summary;
        ceres::Solve(sopts, &problem, &summary);

        BuildNavigationState(kf);
        return {EstimatorUpdateResult::Accepted, "CeresSlidingWindow"};
    }

    MeasurementUpdateReport CreateKeyframe(const core::GnssSolution& gnss, const Eigen::Vector3d& pos_enu) {
        // Finalize IMU preintegration
        ImuPreintegration preint = preint_.Finalize(gnss.timestamp);

        // Create new keyframe, propagating state from previous
        const WindowKeyframe& prev = window_.back();
        WindowKeyframe kf;
        kf.timestamp = gnss.timestamp;

        // IMU propagation for initial guess
        const Eigen::Matrix3d R_prev = prev.Q().toRotationMatrix();
        const Eigen::Vector3d g_enu(0, 0, -kGravityMps2);
        const double dt = preint.integration_time_s;
        const Eigen::Vector3d p_pred = prev.P() + prev.V() * dt + 0.5 * g_enu * dt * dt + R_prev * preint.delta_p;
        const Eigen::Vector3d v_pred = prev.V() + g_enu * dt + R_prev * preint.delta_v;
        const Eigen::Quaterniond q_pred = (prev.Q() * preint.delta_q).normalized();

        kf.pose.Set(p_pred, q_pred);
        kf.SetV(v_pred);
        kf.SetBias(prev.Ba(), prev.Bg());  // propagate biases

        window_.push_back(kf);
        imu_factors_.push_back(preint);

        // Trim window
        while (window_.size() > opts_.max_keyframes + 1) {
            window_.pop_front();
            if (!imu_factors_.empty()) imu_factors_.pop_front();
        }

        // Optimise
        Optimise(gnss, pos_enu);

        // Restart preintegrator from new keyframe
        const auto& new_kf = window_.back();
        preint_.Reset(new_kf.Ba(), new_kf.Bg(), gnss.timestamp);
        last_kf_time_ = gnss.timestamp;

        BuildNavigationState(window_.back());
        return {EstimatorUpdateResult::Accepted, "CeresSlidingWindow"};
    }

    void Optimise(const core::GnssSolution& gnss, const Eigen::Vector3d& pos_enu) {
        ceres::Problem problem;
        const Eigen::Matrix3d R_enu = ltp_->ecef_to_enu_rotation();

        const std::size_t n = window_.size();

        // ── IMU factors between consecutive keyframes ──────────────────────────
        for (std::size_t i = 0; i + 1 < n && i < imu_factors_.size(); ++i) {
            problem.AddResidualBlock(ImuPreintegrationCost::Create(imu_factors_[i]), nullptr, window_[i].pose.data,
                                     window_[i].vel, window_[i].bias, window_[i + 1].pose.data, window_[i + 1].vel,
                                     window_[i + 1].bias);
        }

        // ── Bias random walk ───────────────────────────────────────────────────
        const double ba_sigma = opts_.base.max_buffered_measurements > 0 ? 0.01 : 0.01;
        const double bg_sigma = 0.001;
        for (std::size_t i = 0; i + 1 < n; ++i) {
            problem.AddResidualBlock(BiasRandomWalkCost::Create(ba_sigma, bg_sigma), nullptr, window_[i].bias,
                                     window_[i + 1].bias);
        }

        // ── GNSS factors on latest keyframe ───────────────────────────────────
        auto& latest = window_.back();
        const Eigen::Matrix3d cov_enu = R_enu * gnss.position_covariance_ecef_m2 * R_enu.transpose();
        const Eigen::Matrix3d vel_cov_enu = R_enu * gnss.velocity_covariance_ecef_m2ps2 * R_enu.transpose();

        ceres::LossFunction* pos_loss = nullptr;
        ceres::LossFunction* vel_loss = nullptr;
        if (opts_.gnss_loss == LossFunctionType::Huber) {
            pos_loss = new ceres::HuberLoss(opts_.huber_loss_parameter);
            vel_loss = new ceres::HuberLoss(opts_.huber_loss_parameter);
        }

        problem.AddResidualBlock(GnssPositionCost::Create(pos_enu, cov_enu), pos_loss, latest.pose.data);
        problem.AddResidualBlock(GnssVelocityCost::Create(R_enu * gnss.velocity_ecef_mps.eigen(), vel_cov_enu),
                                 vel_loss, latest.vel);

        // ── Quaternion manifold on all keyframes ───────────────────────────────
        for (auto& kf : window_) {
            FG_CERES_SET_QUAT_MANIFOLD(problem, kf.pose.data);
        }

        // Fix oldest keyframe pose (gauge freedom)
        problem.SetParameterBlockConstant(window_.front().pose.data);
        problem.SetParameterBlockConstant(window_.front().vel);

        // ── Solve ──────────────────────────────────────────────────────────────
        ceres::Solver::Options sopts;
        sopts.logging_type = ceres::SILENT;
        sopts.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        sopts.max_num_iterations = opts_.max_solver_iterations;
        sopts.max_solver_time_in_seconds = opts_.solver_time_budget_s;
        if (opts_.solver_strategy == SolverStrategy::FastOneIteration) sopts.max_num_iterations = 1;

        ceres::Solver::Summary summary;
        ceres::Solve(sopts, &problem, &summary);
    }

    void BuildNavigationState(const WindowKeyframe& kf) {
        latest_state_.timestamp = kf.timestamp;

        // ENU position and velocity
        latest_state_.position_enu_m = core::Vec3<core::EnuFrame>(kf.P().x(), kf.P().y(), kf.P().z());
        latest_state_.velocity_enu_mps = core::Vec3<core::EnuFrame>(kf.V().x(), kf.V().y(), kf.V().z());
        latest_state_.orientation_body_to_enu = kf.Q();

        // ECEF (from ENU + reference)
        const Eigen::Matrix3d R_enu_to_ecef = ltp_->ecef_to_enu_rotation().transpose();
        const Eigen::Vector3d p_ecef = ltp_->origin_ecef_m().eigen() + R_enu_to_ecef * kf.P();
        latest_state_.position_ecef_m = core::Vec3<core::EcefFrame>(p_ecef.x(), p_ecef.y(), p_ecef.z());
        const Eigen::Vector3d v_ecef = R_enu_to_ecef * kf.V();
        latest_state_.velocity_ecef_mps = core::Vec3<core::EcefFrame>(v_ecef.x(), v_ecef.y(), v_ecef.z());

        // Covariance (position + velocity blocks from sliding window; zeros elsewhere)
        latest_state_.covariance.setZero();

        latest_state_.status = core::NavigationStatus::Nominal;
        latest_state_.quality.initialized = true;
    }

    const SlidingWindowState& WindowStateSnapshot() const {
        diagnostic_state_.keyframes.clear();
        diagnostic_state_.imu_factors.assign(imu_factors_.begin(), imu_factors_.end());

        for (const auto& src : window_) {
            Keyframe dst;
            dst.timestamp = src.timestamp;
            const Eigen::Vector3d p = src.P();
            const Eigen::Vector3d v = src.V();
            const Eigen::Vector3d ba = src.Ba();
            const Eigen::Vector3d bg = src.Bg();
            dst.position_enu_m = core::Vec3<core::EnuFrame>(p.x(), p.y(), p.z());
            dst.velocity_enu_mps = core::Vec3<core::EnuFrame>(v.x(), v.y(), v.z());
            dst.orientation_body_to_enu = src.Q();
            dst.accel_bias_mps2 = core::Vec3<core::ImuFrame>(ba.x(), ba.y(), ba.z());
            dst.gyro_bias_radps = core::Vec3<core::ImuFrame>(bg.x(), bg.y(), bg.z());
            diagnostic_state_.keyframes.push_back(std::move(dst));
        }

        return diagnostic_state_;
    }
};

// ── Public API ────────────────────────────────────────────────────────────────

CeresSlidingWindowEstimator::CeresSlidingWindowEstimator(CeresOptions options)
    : options_(std::move(options)), impl_(std::make_unique<Impl>(options_)) {}

// Non-inline dtor (Impl is incomplete at header)
CeresSlidingWindowEstimator::~CeresSlidingWindowEstimator() = default;

EstimatorInfo CeresSlidingWindowEstimator::Info() const {
    return {EstimatorBackend::CeresSlidingWindow, "CeresSlidingWindow", "1.0"};
}

const EstimatorOptions& CeresSlidingWindowEstimator::Options() const { return options_.base; }

bool CeresSlidingWindowEstimator::IsInitialized() const {
    std::unique_lock lk(impl_->mutex_);
    return impl_->initialised_;
}

std::optional<core::NavigationState> CeresSlidingWindowEstimator::LatestState() const {
    std::unique_lock lk(impl_->mutex_);
    if (!impl_->initialised_) return std::nullopt;
    return impl_->latest_state_;
}

MeasurementUpdateReport CeresSlidingWindowEstimator::AddMeasurement(const SensorMeasurement& measurement) {
    std::unique_lock lk(impl_->mutex_);

    if (const auto* imu = std::get_if<core::ImuMeasurement>(&measurement)) return impl_->AddImu(*imu);
    if (const auto* gnss = std::get_if<core::GnssSolution>(&measurement)) return impl_->AddGnss(*gnss);

    return {EstimatorUpdateResult::Rejected};
}

EstimatorUpdateResult CeresSlidingWindowEstimator::ProcessUntil(const core::Timestamp& /*timestamp*/) {
    // The Ceres backend is keyframe-triggered; no explicit propagation step.
    return EstimatorUpdateResult::Accepted;
}

void CeresSlidingWindowEstimator::Reset() {
    std::unique_lock lk(impl_->mutex_);
    impl_ = std::make_unique<Impl>(options_);
    FG_INFO("CeresSlidingWindow | reset");
}

const SlidingWindowState& CeresSlidingWindowEstimator::WindowState() const {
    std::unique_lock lk(impl_->mutex_);
    return impl_->WindowStateSnapshot();
}

}  // namespace falconguide::estimation::ceres_backend

#else  // FALCONGUIDE_HAVE_CERES not defined

#include "falconguide/estimation/backends/ceres/ceres_estimator.hpp"
#include <stdexcept>

namespace falconguide::estimation::ceres_backend {

class CeresSlidingWindowEstimator::Impl {};

CeresSlidingWindowEstimator::~CeresSlidingWindowEstimator() = default;

CeresSlidingWindowEstimator::CeresSlidingWindowEstimator(CeresOptions) {
    throw std::runtime_error(
        "CeresSlidingWindowEstimator: built without Ceres support. "
        "Enable FALCONGUIDE_ENABLE_CERES in CMake.");
}
EstimatorInfo CeresSlidingWindowEstimator::Info() const { return {}; }
const EstimatorOptions& CeresSlidingWindowEstimator::Options() const {
    static EstimatorOptions dummy;
    return dummy;
}
bool CeresSlidingWindowEstimator::IsInitialized() const { return false; }
std::optional<core::NavigationState> CeresSlidingWindowEstimator::LatestState() const { return {}; }
MeasurementUpdateReport CeresSlidingWindowEstimator::AddMeasurement(const SensorMeasurement&) { return {}; }
EstimatorUpdateResult CeresSlidingWindowEstimator::ProcessUntil(const core::Timestamp&) {
    return EstimatorUpdateResult::BackendError;
}
void CeresSlidingWindowEstimator::Reset() {}
const SlidingWindowState& CeresSlidingWindowEstimator::WindowState() const {
    static SlidingWindowState dummy;
    return dummy;
}

}  // namespace falconguide::estimation::ceres_backend

#endif  // FALCONGUIDE_HAVE_CERES
