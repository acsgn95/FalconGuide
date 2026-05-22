#include "falconguide/estimation/navigation_system.hpp"

// EKF
#include "falconguide/estimation/backends/ekf/ekf_estimator.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/aiding_solution.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/airspeed.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/barometer.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/dvl.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/echo_sounder.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/external_odometry.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/external_pose.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/external_velocity.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/gnss_loosely_coupled.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/gnss_tightly_coupled.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/magnetometer.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/optical_flow.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/radar_altimeter.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/range_finder.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/star_tracker.hpp"
#include "falconguide/estimation/backends/ekf/measurement_models/wheel_odometry.hpp"

// UKF
#include "falconguide/estimation/backends/ukf/ukf_estimator.hpp"

// Ceres sliding-window (guarded — header always present, impl gated by define)
#include "falconguide/estimation/backends/ceres/ceres_estimator.hpp"

// GTSAM factor graph (guarded — header always present, impl gated by define)
#include "falconguide/estimation/backends/gtsam/gtsam_estimator.hpp"
#include "falconguide/estimation/backends/ukf/measurement_models/ukf_aiding_solution.hpp"
#include "falconguide/estimation/backends/ukf/measurement_models/ukf_airspeed.hpp"
#include "falconguide/estimation/backends/ukf/measurement_models/ukf_barometer.hpp"
#include "falconguide/estimation/backends/ukf/measurement_models/ukf_dvl.hpp"
#include "falconguide/estimation/backends/ukf/measurement_models/ukf_echo_sounder.hpp"
#include "falconguide/estimation/backends/ukf/measurement_models/ukf_external_odometry.hpp"
#include "falconguide/estimation/backends/ukf/measurement_models/ukf_external_pose.hpp"
#include "falconguide/estimation/backends/ukf/measurement_models/ukf_external_velocity.hpp"
#include "falconguide/estimation/backends/ukf/measurement_models/ukf_gnss_loosely_coupled.hpp"
#include "falconguide/estimation/backends/ukf/measurement_models/ukf_magnetometer.hpp"
#include "falconguide/estimation/backends/ukf/measurement_models/ukf_optical_flow.hpp"
#include "falconguide/estimation/backends/ukf/measurement_models/ukf_radar_altimeter.hpp"
#include "falconguide/estimation/backends/ukf/measurement_models/ukf_star_tracker.hpp"
#include "falconguide/estimation/backends/ukf/measurement_models/ukf_wheel_odometry.hpp"

#include "falconguide/logger/logger.hpp"

namespace falconguide::estimation {

NavigationSystem::NavigationSystem(const NavigationSystemConfig& config) : pipeline_(BuildEstimator(config)) {}

EstimatorPipeline& NavigationSystem::Pipeline() { return pipeline_; }
const EstimatorPipeline& NavigationSystem::Pipeline() const { return pipeline_; }

// ── Dispatch ──────────────────────────────────────────────────────────────────

std::unique_ptr<INavigationEstimator> NavigationSystem::BuildEstimator(const NavigationSystemConfig& config) {
    switch (config.backend) {
        case EstimatorBackendChoice::Ekf:
            return BuildEkfEstimator(config);
        case EstimatorBackendChoice::Ukf:
            return BuildUkfEstimator(config);
        case EstimatorBackendChoice::Ceres:
            return BuildCeresEstimator(config);
        case EstimatorBackendChoice::Gtsam:
            return BuildGtsamEstimator(config);
    }
    return BuildEkfEstimator(config);
}

// ── EKF factory ───────────────────────────────────────────────────────────────

std::unique_ptr<INavigationEstimator> NavigationSystem::BuildEkfEstimator(const NavigationSystemConfig& config) {
    ekf::EkfOptions opts;
    opts.imu_noise = config.imu.noise;
    opts.dead_reckoning_threshold_s = config.dead_reckoning_threshold_s;
    opts.min_imu_dt_s = config.min_imu_dt_s;
    opts.max_imu_dt_s = config.max_imu_dt_s;
    opts.enable_gnss_clock_state = config.gnss.enabled && config.gnss.mode == GnssIntegrationMode::TightlyCoupled;

    auto est = std::make_unique<ekf::EkfEstimator>(std::move(opts));

    if (config.gnss.enabled) {
        if (config.gnss.mode == GnssIntegrationMode::LooselyCoupled)
            est->RegisterMeasurementModel(std::make_unique<ekf::GnssLooselyCoupled>(config.gnss.loosely_coupled));
        else
            est->RegisterMeasurementModel(std::make_unique<ekf::GnssTightlyCoupled>(config.gnss.tightly_coupled));
    }
    if (config.barometer.enabled)
        est->RegisterMeasurementModel(std::make_unique<ekf::Barometer>(config.barometer.options));
    if (config.magnetometer.enabled)
        est->RegisterMeasurementModel(std::make_unique<ekf::Magnetometer>(config.magnetometer.options));
    if (config.wheel_odometry.enabled)
        est->RegisterMeasurementModel(std::make_unique<ekf::WheelOdometry>(config.wheel_odometry.options));
    if (config.airspeed.enabled)
        est->RegisterMeasurementModel(std::make_unique<ekf::Airspeed>(config.airspeed.options));
    if (config.optical_flow.enabled)
        est->RegisterMeasurementModel(std::make_unique<ekf::OpticalFlow>(config.optical_flow.options));
    if (config.radar_altimeter.enabled)
        est->RegisterMeasurementModel(std::make_unique<ekf::RadarAltimeter>(config.radar_altimeter.options));
    if (config.range_finder.enabled)
        est->RegisterMeasurementModel(std::make_unique<ekf::RangeFinder>(config.range_finder.options));
    if (config.dvl.enabled) est->RegisterMeasurementModel(std::make_unique<ekf::Dvl>(config.dvl.options));
    if (config.echo_sounder.enabled)
        est->RegisterMeasurementModel(std::make_unique<ekf::EchoSounder>(config.echo_sounder.options));
    if (config.external_pose.enabled)
        est->RegisterMeasurementModel(std::make_unique<ekf::ExternalPose>(config.external_pose.options));
    if (config.external_velocity.enabled)
        est->RegisterMeasurementModel(std::make_unique<ekf::ExternalVelocity>(config.external_velocity.options));
    if (config.external_odometry.enabled)
        est->RegisterMeasurementModel(std::make_unique<ekf::ExternalOdometry>(config.external_odometry.options));
    if (config.aiding_solution.enabled)
        est->RegisterMeasurementModel(std::make_unique<ekf::AidingSolution>(config.aiding_solution.options));
    if (config.star_tracker.enabled)
        est->RegisterMeasurementModel(std::make_unique<ekf::StarTracker>(config.star_tracker.options));

    return est;
}

// ── UKF factory ───────────────────────────────────────────────────────────────
//
// Maps the common sensor config fields to UKF-specific option structs.
// Sensors without a UKF model (range_finder, gnss tightly coupled) are skipped.
//
std::unique_ptr<INavigationEstimator> NavigationSystem::BuildUkfEstimator(const NavigationSystemConfig& config) {
    if (config.gnss.enabled && config.gnss.mode == GnssIntegrationMode::TightlyCoupled) {
        FG_WARN(
            "NavigationSystem | UKF does not support tightly coupled GNSS; "
            "falling back to loosely coupled");
    }

    ukf::UkfOptions opts;
    opts.sigma_params = config.ukf_sigma_params;
    opts.imu_noise.accel_noise_density_mps2_per_sqrthz = config.imu.noise.accel_noise_density_mps2_per_sqrthz;
    opts.imu_noise.gyro_noise_density_radps_per_sqrthz = config.imu.noise.gyro_noise_density_radps_per_sqrthz;
    opts.imu_noise.accel_random_walk_mps3_per_sqrthz = config.imu.noise.accel_random_walk_mps3_per_sqrthz;
    opts.imu_noise.gyro_random_walk_radps2_per_sqrthz = config.imu.noise.gyro_random_walk_radps2_per_sqrthz;
    opts.dead_reckoning_threshold_s = config.dead_reckoning_threshold_s;
    opts.min_imu_dt_s = config.min_imu_dt_s;
    opts.max_imu_dt_s = config.max_imu_dt_s;

    auto est = std::make_unique<ukf::UkfEstimator>(std::move(opts));

    // GNSS — only loosely coupled available for UKF
    if (config.gnss.enabled) {
        ukf::UkfGnssLooselyCoupledOptions g;
        g.position_sigma_m = config.gnss.loosely_coupled.position_sigma_m;
        g.velocity_sigma_mps = config.gnss.loosely_coupled.velocity_sigma_mps;
        est->RegisterMeasurementModel(std::make_unique<ukf::UkfGnssLooselyCoupled>(g));
    }

    if (config.barometer.enabled) {
        ukf::UkfBarometerOptions b;
        b.altitude_sigma_m = config.barometer.options.altitude_sigma_m;
        b.isa_sea_level_pressure_pa = config.barometer.options.isa_sea_level_pressure_pa;
        b.innovation_gate = config.barometer.options.innovation_gate;
        est->RegisterMeasurementModel(std::make_unique<ukf::UkfBarometer>(b));
    }

    if (config.magnetometer.enabled) {
        ukf::UkfMagnetometerOptions m;
        m.reference_field_enu_tesla = config.magnetometer.options.reference_field_enu_tesla;
        m.sigma_tesla = config.magnetometer.options.sigma_tesla;
        m.innovation_gate = config.magnetometer.options.innovation_gate;
        est->RegisterMeasurementModel(std::make_unique<ukf::UkfMagnetometer>(m));
    }

    if (config.wheel_odometry.enabled) {
        ukf::UkfWheelOdometryOptions w;
        w.sigma_linear_mps = config.wheel_odometry.options.sigma_linear_mps;
        w.innovation_gate = config.wheel_odometry.options.innovation_gate;
        est->RegisterMeasurementModel(std::make_unique<ukf::UkfWheelOdometry>(w));
    }

    if (config.airspeed.enabled) {
        ukf::UkfAirspeedOptions a;
        a.wind_enu_mps = config.airspeed.options.wind_enu_mps;
        a.sigma_mps = config.airspeed.options.sigma_mps;
        a.innovation_gate = config.airspeed.options.innovation_gate;
        est->RegisterMeasurementModel(std::make_unique<ukf::UkfAirspeed>(a));
    }

    if (config.optical_flow.enabled) {
        ukf::UkfOpticalFlowOptions o;
        o.fallback_altitude_m = config.optical_flow.options.fallback_altitude_m;
        o.sigma_radps = config.optical_flow.options.sigma_radps;
        o.innovation_gate = config.optical_flow.options.innovation_gate;
        est->RegisterMeasurementModel(std::make_unique<ukf::UkfOpticalFlow>(o));
    }

    if (config.radar_altimeter.enabled) {
        ukf::UkfRadarAltimeterOptions r;
        r.terrain_elevation_m = config.radar_altimeter.options.terrain_elevation_m;
        r.sigma_m = config.radar_altimeter.options.sigma_m;
        r.innovation_gate = config.radar_altimeter.options.innovation_gate;
        est->RegisterMeasurementModel(std::make_unique<ukf::UkfRadarAltimeter>(r));
    }

    // range_finder: no UKF equivalent, skipped.

    if (config.dvl.enabled) {
        ukf::UkfDvlOptions d;
        d.dvl_to_body_rotation = config.dvl.options.dvl_to_body_rotation;
        d.bottom_track_only = config.dvl.options.bottom_track_only;
        d.sigma_mps = config.dvl.options.sigma_mps;
        d.innovation_gate = config.dvl.options.innovation_gate;
        est->RegisterMeasurementModel(std::make_unique<ukf::UkfDvl>(d));
    }

    if (config.echo_sounder.enabled) {
        ukf::UkfEchoSounderOptions e;
        e.sound_speed_mps = config.echo_sounder.options.sound_speed_mps;
        e.depth_origin_m = config.echo_sounder.options.depth_origin_m;
        e.sigma_m = config.echo_sounder.options.sigma_m;
        e.innovation_gate = config.echo_sounder.options.innovation_gate;
        est->RegisterMeasurementModel(std::make_unique<ukf::UkfEchoSounder>(e));
    }

    if (config.external_pose.enabled) {
        ukf::UkfExternalPoseOptions ep;
        ep.use_position = config.external_pose.options.use_position;
        ep.use_orientation = config.external_pose.options.use_orientation;
        ep.position_sigma_m = config.external_pose.options.position_sigma_m;
        ep.orientation_sigma_rad = config.external_pose.options.orientation_sigma_rad;
        ep.innovation_gate = config.external_pose.options.innovation_gate;
        est->RegisterMeasurementModel(std::make_unique<ukf::UkfExternalPose>(ep));
    }

    if (config.external_velocity.enabled) {
        ukf::UkfExternalVelocityOptions ev;
        ev.sigma_linear_mps = config.external_velocity.options.sigma_linear_mps;
        ev.innovation_gate = config.external_velocity.options.innovation_gate;
        est->RegisterMeasurementModel(std::make_unique<ukf::UkfExternalVelocity>(ev));
    }

    if (config.external_odometry.enabled) {
        ukf::UkfExternalOdometryOptions eo;
        eo.use_position = config.external_odometry.options.use_position;
        eo.use_velocity = config.external_odometry.options.use_velocity;
        eo.use_orientation = config.external_odometry.options.use_orientation;
        eo.position_sigma_m = config.external_odometry.options.position_sigma_m;
        eo.velocity_sigma_mps = config.external_odometry.options.velocity_sigma_mps;
        eo.orientation_sigma_rad = config.external_odometry.options.orientation_sigma_rad;
        eo.innovation_gate = config.external_odometry.options.innovation_gate;
        est->RegisterMeasurementModel(std::make_unique<ukf::UkfExternalOdometry>(eo));
    }

    if (config.aiding_solution.enabled) {
        ukf::UkfAidingSolutionOptions as;
        as.min_match_score = config.aiding_solution.options.min_match_score;
        as.position_sigma_m = config.aiding_solution.options.position_sigma_m;
        as.velocity_sigma_mps = config.aiding_solution.options.velocity_sigma_mps;
        as.attitude_sigma_rad = config.aiding_solution.options.attitude_sigma_rad;
        as.innovation_gate = config.aiding_solution.options.innovation_gate;
        est->RegisterMeasurementModel(std::make_unique<ukf::UkfAidingSolution>(as));
    }

    if (config.star_tracker.enabled) {
        ukf::UkfStarTrackerOptions st;
        st.star_tracker_to_body = config.star_tracker.options.star_tracker_to_body;
        st.sigma_yaw_rad = config.star_tracker.options.sigma_yaw_rad;
        st.sigma_pitch_rad = config.star_tracker.options.sigma_pitch_rad;
        st.sigma_roll_rad = config.star_tracker.options.sigma_roll_rad;
        st.use_pitch_roll = config.star_tracker.options.use_pitch_roll;
        st.innovation_gate = config.star_tracker.options.innovation_gate;
        est->RegisterMeasurementModel(std::make_unique<ukf::UkfStarTracker>(st));
    }

    return est;
}

// ── Ceres factory ─────────────────────────────────────────────────────────────

std::unique_ptr<INavigationEstimator> NavigationSystem::BuildCeresEstimator(
    [[maybe_unused]] const NavigationSystemConfig& config) {
    ceres_backend::CeresOptions opts;
    return std::make_unique<ceres_backend::CeresSlidingWindowEstimator>(std::move(opts));
}

// ── GTSAM factory ─────────────────────────────────────────────────────────────

std::unique_ptr<INavigationEstimator> NavigationSystem::BuildGtsamEstimator(
    [[maybe_unused]] const NavigationSystemConfig& config) {
    gtsam_backend::GtsamOptions opts;
    return std::make_unique<gtsam_backend::GtsamFactorGraphEstimator>(std::move(opts));
}

}  // namespace falconguide::estimation
