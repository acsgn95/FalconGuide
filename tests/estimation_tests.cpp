#include "falconguide/estimation/backends/ekf/ekf_estimator.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/barometer.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/gnss_loosely_coupled.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/magnetometer.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/wheel_odometry.hpp"
#include "falconguide/estimation/backends/ukf/ukf_estimator.hpp"
#include "falconguide/estimation/backends/ukf/measurement_models/ukf_barometer.hpp"
#include "falconguide/estimation/backends/ukf/measurement_models/ukf_gnss_loosely_coupled.hpp"
#include "falconguide/estimation/backends/ukf/measurement_models/ukf_magnetometer.hpp"
#include "falconguide/estimation/backends/ukf/measurement_models/ukf_wheel_odometry.hpp"
#include "falconguide/estimation/estimator_interface.hpp"
#include "falconguide/estimation/navigation_system.hpp"
#include "falconguide/estimation/navigation_system_config.hpp"
#include "falconguide/core/coordinates.hpp"
#include "falconguide/core/math.hpp"
#include "falconguide/core/sensors/environment.hpp"
#include "falconguide/core/sensors/gnss.hpp"
#include "falconguide/core/sensors/imu.hpp"
#include "falconguide/core/sensors/odometry.hpp"
#include "falconguide/logger/logger.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <thread>

using namespace falconguide;
using namespace falconguide::estimation;
using namespace falconguide::core;

// ── Helpers ───────────────────────────────────────────────────────────────────

static Timestamp MakeTimestamp(std::int64_t ns) {
  Timestamp t;
  t.has_steady = true;
  t.steady = MonotonicTime(std::chrono::nanoseconds(ns));
  return t;
}

static GnssSolution MakeGnss(std::int64_t ns, double lat_rad, double lon_rad, double alt_m) {
  GnssSolution g;
  g.timestamp  = MakeTimestamp(ns);
  g.fix_type   = GnssFixType::Single;
  g.validity   = MeasurementValidity::Valid;
  g.position_ecef_m = LlaToEcef(Lla{lat_rad, lon_rad, alt_m});
  g.velocity_ecef_mps = Vec3<EcefFrame>(0.0, 0.0, 0.0);
  g.position_covariance_ecef_m2 = Eigen::Matrix3d::Identity() * 4.0;   // 2 m σ
  g.velocity_covariance_ecef_m2ps2 = Eigen::Matrix3d::Identity() * 0.25;
  return g;
}

static ImuMeasurement MakeImu(std::int64_t ns) {
  ImuMeasurement imu;
  imu.timestamp = MakeTimestamp(ns);
  // Stationary: gravity downward in body frame (-Z body = -Z ENU)
  imu.specific_force_mps2 = Vec3<ImuFrame>(0.0, 0.0, -9.80665);
  imu.angular_rate_radps  = Vec3<ImuFrame>(0.0, 0.0, 0.0);
  imu.validity = MeasurementValidity::Valid;
  return imu;
}

static bool Near(double a, double b, double tol) {
  return std::abs(a - b) <= tol;
}

static BarometerMeasurement MakeBaro(std::int64_t ns, double altitude_m) {
  BarometerMeasurement baro;
  baro.timestamp  = MakeTimestamp(ns);
  baro.altitude_m = altitude_m;
  // pressure from barometric formula (ISA, rough)
  baro.pressure_pa = 101325.0 * std::pow(1.0 - 0.0000225577 * altitude_m, 5.25588);
  baro.validity   = MeasurementValidity::Valid;
  return baro;
}

static MagnetometerMeasurement MakeMag(std::int64_t ns, Eigen::Vector3d field_enu) {
  MagnetometerMeasurement mag;
  mag.timestamp = MakeTimestamp(ns);
  // sensor frame aligned with body; body nominally aligned with ENU after GNSS init
  mag.magnetic_field_tesla = Vec3<MagnetometerFrame>(field_enu.x(), field_enu.y(), field_enu.z());
  mag.validity = MeasurementValidity::Valid;
  return mag;
}

static WheelOdometryMeasurement MakeWheelOdo(std::int64_t ns, double forward_mps) {
  WheelOdometryMeasurement odo;
  odo.timestamp = MakeTimestamp(ns);
  odo.linear_velocity_body_mps  = Vec3<BodyFrame>(forward_mps, 0.0, 0.0);
  odo.angular_rate_body_radps   = Vec3<BodyFrame>(0.0, 0.0, 0.0);
  odo.covariance = Eigen::Matrix<double, 6, 6>::Identity() * 0.01;
  odo.validity   = MeasurementValidity::Valid;
  return odo;
}

// ── EKF Tests ─────────────────────────────────────────────────────────────────

static void TestEkfNotInitializedOnConstruct() {
  ekf::EkfEstimator est;
  assert(!est.IsInitialized());
  assert(!est.LatestState().has_value());
}

static void TestEkfInitFromGnss() {
  ekf::EkfEstimator est;
  est.RegisterMeasurementModel(std::make_unique<ekf::GnssLooselyCoupled>());

  const auto gnss = MakeGnss(1'000'000'000LL, DegToRad(48.0), DegToRad(11.0), 500.0);
  const auto report = est.AddMeasurement(gnss);

  assert(report.result == EstimatorUpdateResult::Accepted);
  assert(est.IsInitialized());

  const auto state = est.LatestState();
  assert(state.has_value());
  assert(state->status == NavigationStatus::DeadReckoning);  // no aiding yet after init
  assert(state->quality.initialized);
}

static void TestEkfImuBuffered() {
  ekf::EkfEstimator est;

  // IMU before init should be buffered, not rejected
  const auto r = est.AddMeasurement(MakeImu(500'000'000LL));
  assert(r.result == EstimatorUpdateResult::Buffered);
}

static void TestEkfImuPropagation() {
  ekf::EkfEstimator est;
  est.RegisterMeasurementModel(std::make_unique<ekf::GnssLooselyCoupled>());

  // Init at t=1s
  est.AddMeasurement(MakeGnss(1'000'000'000LL, DegToRad(48.0), DegToRad(11.0), 500.0));
  const auto state0 = est.LatestState();
  assert(state0.has_value());

  // Push IMU at t=1.01s, t=1.02s
  est.AddMeasurement(MakeImu(1'010'000'000LL));
  est.AddMeasurement(MakeImu(1'020'000'000LL));

  // Force propagation to t=1.02s
  const auto result = est.ProcessUntil(MakeTimestamp(1'020'000'000LL));
  assert(result == EstimatorUpdateResult::Accepted);

  const auto state1 = est.LatestState();
  assert(state1.has_value());
  // Stationary with gravity: position should barely move
  const double pos_drift = (state1->position_enu_m.eigen() - state0->position_enu_m.eigen()).norm();
  assert(pos_drift < 0.01);
}

static void TestEkfGnssUpdate() {
  ekf::EkfEstimator est;
  est.RegisterMeasurementModel(std::make_unique<ekf::GnssLooselyCoupled>());

  // Init
  est.AddMeasurement(MakeGnss(1'000'000'000LL, DegToRad(48.0), DegToRad(11.0), 500.0));

  // Push some IMU
  for (int i = 1; i <= 10; ++i)
    est.AddMeasurement(MakeImu(1'000'000'000LL + i * 10'000'000LL));

  // Second GNSS update (100 ms later, same position)
  const auto gnss2 = MakeGnss(1'100'000'000LL, DegToRad(48.0), DegToRad(11.0), 500.0);
  const auto report = est.AddMeasurement(gnss2);

  assert(report.result == EstimatorUpdateResult::Accepted);
  assert(report.model_name == "GnssLooselyCoupled");
  assert(report.correction_norm.has_value());

  const auto state = est.LatestState();
  assert(state.has_value());
  assert(state->status == NavigationStatus::Nominal);
}

static void TestEkfOutOfOrderImuDiscarded() {
  ekf::EkfEstimator est;
  est.RegisterMeasurementModel(std::make_unique<ekf::GnssLooselyCoupled>());
  est.AddMeasurement(MakeGnss(1'000'000'000LL, DegToRad(48.0), DegToRad(11.0), 500.0));

  // Push t=1.1s then t=1.0s (out of order)
  est.AddMeasurement(MakeImu(1'100'000'000LL));
  const auto r = est.AddMeasurement(MakeImu(1'000'000'000LL));
  assert(r.result == EstimatorUpdateResult::OutOfOrder);
}

static void TestEkfDeadReckoningStatus() {
  ekf::EkfOptions opts;
  opts.dead_reckoning_threshold_s = 0.05;  // 50 ms threshold for test
  ekf::EkfEstimator est(opts);
  est.RegisterMeasurementModel(std::make_unique<ekf::GnssLooselyCoupled>());

  // Init + one GNSS update
  est.AddMeasurement(MakeGnss(1'000'000'000LL, DegToRad(48.0), DegToRad(11.0), 500.0));
  est.AddMeasurement(MakeGnss(1'100'000'000LL, DegToRad(48.0), DegToRad(11.0), 500.0));

  // Push IMU well past the threshold (200 ms later, no aiding)
  for (int i = 1; i <= 20; ++i)
    est.AddMeasurement(MakeImu(1'100'000'000LL + i * 10'000'000LL));
  est.ProcessUntil(MakeTimestamp(1'300'000'000LL));

  const auto state = est.LatestState();
  assert(state.has_value());
  assert(state->status == NavigationStatus::DeadReckoning);
}

static void TestEkfReset() {
  ekf::EkfEstimator est;
  est.RegisterMeasurementModel(std::make_unique<ekf::GnssLooselyCoupled>());
  est.AddMeasurement(MakeGnss(1'000'000'000LL, DegToRad(48.0), DegToRad(11.0), 500.0));
  assert(est.IsInitialized());

  est.Reset();
  assert(!est.IsInitialized());
  assert(!est.LatestState().has_value());
}

static void TestEkfCovarianceShrinks() {
  ekf::EkfEstimator est;
  est.RegisterMeasurementModel(std::make_unique<ekf::GnssLooselyCoupled>());
  est.AddMeasurement(MakeGnss(1'000'000'000LL, DegToRad(48.0), DegToRad(11.0), 500.0));

  // Push IMU (covariance grows)
  for (int i = 1; i <= 50; ++i)
    est.AddMeasurement(MakeImu(1'000'000'000LL + i * 10'000'000LL));

  const auto state_before = est.LatestState();
  const double pos_var_before = state_before->covariance.block<3,3>(0,0).trace();

  // GNSS update (covariance shrinks)
  est.AddMeasurement(MakeGnss(1'510'000'000LL, DegToRad(48.0), DegToRad(11.0), 500.0));
  const auto state_after = est.LatestState();
  const double pos_var_after = state_after->covariance.block<3,3>(0,0).trace();

  assert(pos_var_after < pos_var_before);
}

// ── UKF Tests ─────────────────────────────────────────────────────────────────

static void TestUkfNotInitializedOnConstruct() {
  ukf::UkfEstimator est;
  assert(!est.IsInitialized());
  assert(!est.LatestState().has_value());
}

static void TestUkfInitFromGnss() {
  ukf::UkfEstimator est;
  est.RegisterMeasurementModel(std::make_unique<ukf::UkfGnssLooselyCoupled>());

  const auto gnss = MakeGnss(1'000'000'000LL, DegToRad(48.0), DegToRad(11.0), 500.0);
  const auto report = est.AddMeasurement(gnss);

  assert(report.result == EstimatorUpdateResult::Accepted);
  assert(est.IsInitialized());

  const auto state = est.LatestState();
  assert(state.has_value());
  assert(state->quality.initialized);
}

static void TestUkfGnssUpdate() {
  ukf::UkfEstimator est;
  est.RegisterMeasurementModel(std::make_unique<ukf::UkfGnssLooselyCoupled>());

  est.AddMeasurement(MakeGnss(1'000'000'000LL, DegToRad(48.0), DegToRad(11.0), 500.0));

  for (int i = 1; i <= 10; ++i)
    est.AddMeasurement(MakeImu(1'000'000'000LL + i * 10'000'000LL));

  const auto gnss2 = MakeGnss(1'100'000'000LL, DegToRad(48.0), DegToRad(11.0), 500.0);
  const auto report = est.AddMeasurement(gnss2);

  assert(report.result == EstimatorUpdateResult::Accepted);
  assert(report.correction_norm.has_value());

  const auto state = est.LatestState();
  assert(state.has_value());
  assert(state->status == NavigationStatus::Nominal);
}

static void TestUkfCovarianceShrinks() {
  ukf::UkfEstimator est;
  est.RegisterMeasurementModel(std::make_unique<ukf::UkfGnssLooselyCoupled>());
  est.AddMeasurement(MakeGnss(1'000'000'000LL, DegToRad(48.0), DegToRad(11.0), 500.0));

  for (int i = 1; i <= 50; ++i)
    est.AddMeasurement(MakeImu(1'000'000'000LL + i * 10'000'000LL));

  const auto state_before = est.LatestState();
  const double pos_var_before = state_before->covariance.block<3,3>(0,0).trace();

  est.AddMeasurement(MakeGnss(1'510'000'000LL, DegToRad(48.0), DegToRad(11.0), 500.0));
  const auto state_after = est.LatestState();
  const double pos_var_after = state_after->covariance.block<3,3>(0,0).trace();

  assert(pos_var_after < pos_var_before);
}

static void TestUkfReset() {
  ukf::UkfEstimator est;
  est.RegisterMeasurementModel(std::make_unique<ukf::UkfGnssLooselyCoupled>());
  est.AddMeasurement(MakeGnss(1'000'000'000LL, DegToRad(48.0), DegToRad(11.0), 500.0));
  assert(est.IsInitialized());

  est.Reset();
  assert(!est.IsInitialized());
}

// ── EKF Barometer Tests ───────────────────────────────────────────────────────

static void TestEkfBarometerAccepted() {
  ekf::EkfEstimator est;
  est.RegisterMeasurementModel(std::make_unique<ekf::GnssLooselyCoupled>());
  est.RegisterMeasurementModel(std::make_unique<ekf::Barometer>());

  est.AddMeasurement(MakeGnss(1'000'000'000LL, DegToRad(48.0), DegToRad(11.0), 500.0));
  for (int i = 1; i <= 5; ++i)
    est.AddMeasurement(MakeImu(1'000'000'000LL + i * 10'000'000LL));

  const auto report = est.AddMeasurement(MakeBaro(1'060'000'000LL, 500.0));
  assert(report.result == EstimatorUpdateResult::Accepted);
  assert(report.model_name == "Barometer");
}

static void TestEkfBarometerCovarianceShrinks() {
  ekf::EkfEstimator est;
  est.RegisterMeasurementModel(std::make_unique<ekf::GnssLooselyCoupled>());
  est.RegisterMeasurementModel(std::make_unique<ekf::Barometer>());

  est.AddMeasurement(MakeGnss(1'000'000'000LL, DegToRad(48.0), DegToRad(11.0), 500.0));
  for (int i = 1; i <= 30; ++i)
    est.AddMeasurement(MakeImu(1'000'000'000LL + i * 10'000'000LL));

  const auto state_before = est.LatestState();
  const double var_before = state_before->covariance(2, 2);  // altitude variance

  est.AddMeasurement(MakeBaro(1'310'000'000LL, 500.0));
  const auto state_after = est.LatestState();
  const double var_after = state_after->covariance(2, 2);

  assert(var_after < var_before);
}

// ── EKF Magnetometer Tests ────────────────────────────────────────────────────

static void TestEkfMagnetometerAccepted() {
  ekf::MagnetometerOptions mag_opts;
  mag_opts.reference_field_enu_tesla = Eigen::Vector3d(0.0, 2.0e-5, -4.3e-5);

  ekf::EkfEstimator est;
  est.RegisterMeasurementModel(std::make_unique<ekf::GnssLooselyCoupled>());
  est.RegisterMeasurementModel(std::make_unique<ekf::Magnetometer>(mag_opts));

  est.AddMeasurement(MakeGnss(1'000'000'000LL, DegToRad(48.0), DegToRad(11.0), 500.0));
  for (int i = 1; i <= 5; ++i)
    est.AddMeasurement(MakeImu(1'000'000'000LL + i * 10'000'000LL));

  const auto report = est.AddMeasurement(
      MakeMag(1'060'000'000LL, Eigen::Vector3d(0.0, 2.0e-5, -4.3e-5)));
  assert(report.result == EstimatorUpdateResult::Accepted);
  assert(report.model_name == "Magnetometer");
}

// ── EKF Wheel Odometry Tests ──────────────────────────────────────────────────

static void TestEkfWheelOdometryAccepted() {
  ekf::EkfEstimator est;
  est.RegisterMeasurementModel(std::make_unique<ekf::GnssLooselyCoupled>());
  est.RegisterMeasurementModel(std::make_unique<ekf::WheelOdometry>());

  est.AddMeasurement(MakeGnss(1'000'000'000LL, DegToRad(48.0), DegToRad(11.0), 500.0));
  for (int i = 1; i <= 5; ++i)
    est.AddMeasurement(MakeImu(1'000'000'000LL + i * 10'000'000LL));

  const auto report = est.AddMeasurement(MakeWheelOdo(1'060'000'000LL, 0.0));
  assert(report.result == EstimatorUpdateResult::Accepted);
  assert(report.model_name == "WheelOdometry");
}

static void TestEkfWheelOdometryVelocityCorrected() {
  ekf::EkfEstimator est;
  est.RegisterMeasurementModel(std::make_unique<ekf::GnssLooselyCoupled>());
  est.RegisterMeasurementModel(std::make_unique<ekf::WheelOdometry>());

  // Init stationary
  est.AddMeasurement(MakeGnss(1'000'000'000LL, DegToRad(48.0), DegToRad(11.0), 500.0));

  // IMU stream
  for (int i = 1; i <= 10; ++i)
    est.AddMeasurement(MakeImu(1'000'000'000LL + i * 10'000'000LL));

  // Odometry says stationary — velocity should stay near zero
  est.AddMeasurement(MakeWheelOdo(1'110'000'000LL, 0.0));

  const auto state = est.LatestState();
  assert(state.has_value());
  const double speed = state->velocity_enu_mps.eigen().norm();
  assert(speed < 0.5);
}

// ── UKF Barometer Tests ───────────────────────────────────────────────────────

static void TestUkfBarometerAccepted() {
  ukf::UkfEstimator est;
  est.RegisterMeasurementModel(std::make_unique<ukf::UkfGnssLooselyCoupled>());
  est.RegisterMeasurementModel(std::make_unique<ukf::UkfBarometer>());

  est.AddMeasurement(MakeGnss(1'000'000'000LL, DegToRad(48.0), DegToRad(11.0), 500.0));
  for (int i = 1; i <= 5; ++i)
    est.AddMeasurement(MakeImu(1'000'000'000LL + i * 10'000'000LL));

  const auto report = est.AddMeasurement(MakeBaro(1'060'000'000LL, 500.0));
  assert(report.result == EstimatorUpdateResult::Accepted);
}

static void TestUkfBarometerCovarianceShrinks() {
  ukf::UkfEstimator est;
  est.RegisterMeasurementModel(std::make_unique<ukf::UkfGnssLooselyCoupled>());
  est.RegisterMeasurementModel(std::make_unique<ukf::UkfBarometer>());

  est.AddMeasurement(MakeGnss(1'000'000'000LL, DegToRad(48.0), DegToRad(11.0), 500.0));
  for (int i = 1; i <= 30; ++i)
    est.AddMeasurement(MakeImu(1'000'000'000LL + i * 10'000'000LL));

  const auto state_before = est.LatestState();
  const double var_before = state_before->covariance(2, 2);

  est.AddMeasurement(MakeBaro(1'310'000'000LL, 500.0));
  const auto state_after = est.LatestState();
  const double var_after = state_after->covariance(2, 2);

  assert(var_after < var_before);
}

// ── UKF Magnetometer Tests ────────────────────────────────────────────────────

static void TestUkfMagnetometerAccepted() {
  ukf::UkfMagnetometerOptions mag_opts;
  mag_opts.reference_field_enu_tesla = Eigen::Vector3d(0.0, 2.0e-5, -4.3e-5);

  ukf::UkfEstimator est;
  est.RegisterMeasurementModel(std::make_unique<ukf::UkfGnssLooselyCoupled>());
  est.RegisterMeasurementModel(std::make_unique<ukf::UkfMagnetometer>(mag_opts));

  est.AddMeasurement(MakeGnss(1'000'000'000LL, DegToRad(48.0), DegToRad(11.0), 500.0));
  for (int i = 1; i <= 5; ++i)
    est.AddMeasurement(MakeImu(1'000'000'000LL + i * 10'000'000LL));

  const auto report = est.AddMeasurement(
      MakeMag(1'060'000'000LL, Eigen::Vector3d(0.0, 2.0e-5, -4.3e-5)));
  assert(report.result == EstimatorUpdateResult::Accepted);
}

// ── UKF Wheel Odometry Tests ──────────────────────────────────────────────────

static void TestUkfWheelOdometryAccepted() {
  ukf::UkfEstimator est;
  est.RegisterMeasurementModel(std::make_unique<ukf::UkfGnssLooselyCoupled>());
  est.RegisterMeasurementModel(std::make_unique<ukf::UkfWheelOdometry>());

  est.AddMeasurement(MakeGnss(1'000'000'000LL, DegToRad(48.0), DegToRad(11.0), 500.0));
  for (int i = 1; i <= 5; ++i)
    est.AddMeasurement(MakeImu(1'000'000'000LL + i * 10'000'000LL));

  const auto report = est.AddMeasurement(MakeWheelOdo(1'060'000'000LL, 0.0));
  assert(report.result == EstimatorUpdateResult::Accepted);
}

// ── NavigationSystem Tests ────────────────────────────────────────────────────

static void TestNavigationSystemEkf() {
  NavigationSystemConfig cfg;
  cfg.backend       = EstimatorBackendChoice::Ekf;
  cfg.gnss.enabled  = true;

  NavigationSystem nav(cfg);
  assert(!nav.Pipeline().IsRunning());
  assert(nav.Pipeline().Estimator().Info().backend == EstimatorBackend::Ekf);
}

static void TestNavigationSystemUkf() {
  NavigationSystemConfig cfg;
  cfg.backend       = EstimatorBackendChoice::Ukf;
  cfg.gnss.enabled  = true;

  NavigationSystem nav(cfg);
  assert(nav.Pipeline().Estimator().Info().name == "UkfEstimator");
}

static void TestNavigationSystemPipeline() {
  NavigationSystemConfig cfg;
  cfg.backend      = EstimatorBackendChoice::Ekf;
  cfg.gnss.enabled = true;

  NavigationSystem nav(cfg);

  bool state_received = false;
  struct Observer : INavigationObserver {
    bool& received;
    explicit Observer(bool& r) : received(r) {}
    void OnNavigationState(std::shared_ptr<const NavigationState>) override {
      received = true;
    }
  } obs(state_received);

  nav.Pipeline().RegisterObserver(&obs);
  nav.Pipeline().Start();

  // Push GNSS init + IMU + second GNSS (triggers observer)
  nav.Pipeline().Push(MakeGnss(1'000'000'000LL, DegToRad(48.0), DegToRad(11.0), 500.0));
  for (int i = 1; i <= 5; ++i)
    nav.Pipeline().Push(MakeImu(1'000'000'000LL + i * 10'000'000LL));
  nav.Pipeline().Push(MakeGnss(1'060'000'000LL, DegToRad(48.0), DegToRad(11.0), 500.0));

  // Give the pipeline thread time to process
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  nav.Pipeline().Stop();

  assert(state_received);
}

// ── main ──────────────────────────────────────────────────────────────────────

int main() {
  falconguide::log::LoggerConfig log_cfg;
  log_cfg.console = false;  // suppress output during tests
  falconguide::log::Init(log_cfg);

  // EKF
  TestEkfNotInitializedOnConstruct();
  TestEkfInitFromGnss();
  TestEkfImuBuffered();
  TestEkfImuPropagation();
  TestEkfGnssUpdate();
  TestEkfOutOfOrderImuDiscarded();
  TestEkfDeadReckoningStatus();
  TestEkfReset();
  TestEkfCovarianceShrinks();

  // EKF — barometer
  TestEkfBarometerAccepted();
  TestEkfBarometerCovarianceShrinks();

  // EKF — magnetometer
  TestEkfMagnetometerAccepted();

  // EKF — wheel odometry
  TestEkfWheelOdometryAccepted();
  TestEkfWheelOdometryVelocityCorrected();

  // UKF
  TestUkfNotInitializedOnConstruct();
  TestUkfInitFromGnss();
  TestUkfGnssUpdate();
  TestUkfCovarianceShrinks();
  TestUkfReset();

  // UKF — barometer
  TestUkfBarometerAccepted();
  TestUkfBarometerCovarianceShrinks();

  // UKF — magnetometer
  TestUkfMagnetometerAccepted();

  // UKF — wheel odometry
  TestUkfWheelOdometryAccepted();

  // NavigationSystem
  TestNavigationSystemEkf();
  TestNavigationSystemUkf();
  TestNavigationSystemPipeline();

  falconguide::log::Shutdown();
  return 0;
}
