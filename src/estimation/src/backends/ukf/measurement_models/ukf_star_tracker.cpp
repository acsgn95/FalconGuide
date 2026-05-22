#include "falconguide/estimation/backends/ukf/measurement_models/ukf_star_tracker.hpp"

#include "falconguide/core/math.hpp"

#include <variant>

namespace falconguide::estimation::ukf {

static Eigen::Quaterniond QuaternionFromStarTracker(double lon_rad, double lat_rad, double yaw_rad,
                                                    const Eigen::Quaterniond& sensor_to_body) {
    const Eigen::Quaterniond q_lon(Eigen::AngleAxisd(lon_rad, Eigen::Vector3d::UnitZ()));
    const Eigen::Quaterniond q_lat(Eigen::AngleAxisd(core::kPi / 2.0 - lat_rad, Eigen::Vector3d::UnitY()));
    const Eigen::Quaterniond q_yaw(Eigen::AngleAxisd(-yaw_rad, Eigen::Vector3d::UnitZ()));
    return (q_lon * q_lat * q_yaw * sensor_to_body).normalized();
}

UkfStarTracker::UkfStarTracker(UkfStarTrackerOptions options) : options_(std::move(options)) {}

bool UkfStarTracker::CanHandle(const SensorMeasurement& measurement) const {
    return std::holds_alternative<core::StarTrackerMeasurement>(measurement);
}

int UkfStarTracker::MeasurementDim(const SensorMeasurement& measurement) const {
    const auto& st = std::get<core::StarTrackerMeasurement>(measurement);
    return (options_.use_pitch_roll && st.pitch_rad && st.roll_rad) ? 3 : 1;
}

// Predict returns LogMapSo3(q_body_to_enu) — rotation vector in tangent space.
Eigen::VectorXd UkfStarTracker::Predict(const NominalState& sigma_state, const UkfUpdateContext& /*ctx*/) const {
    // For the UKF the measurement dimension may be 1 or 3 depending on the
    // last measurement, but Predict must return 3 to match S / Pxz dimensions.
    // The Innovation() override selects the relevant component(s).
    return core::LogMapSo3(sigma_state.orientation_body_to_enu);
}

std::optional<Eigen::VectorXd> UkfStarTracker::Observe(const SensorMeasurement& measurement,
                                                       const UkfUpdateContext& /*ctx*/) const {
    const auto& st = std::get<core::StarTrackerMeasurement>(measurement);
    if (st.validity != core::MeasurementValidity::Valid) return std::nullopt;

    const Eigen::Quaterniond q_obs =
        QuaternionFromStarTracker(st.longitude_rad, st.latitude_rad, st.yaw_rad, options_.star_tracker_to_body);
    return core::LogMapSo3(q_obs);
}

Eigen::MatrixXd UkfStarTracker::NoiseCovariance(const SensorMeasurement& measurement,
                                                const UkfUpdateContext& /*ctx*/) const {
    const auto& st = std::get<core::StarTrackerMeasurement>(measurement);
    const bool full = options_.use_pitch_roll && st.pitch_rad && st.roll_rad;

    if (full) {
        Eigen::Matrix3d R;
        R.diagonal() << options_.sigma_roll_rad * options_.sigma_roll_rad,
            options_.sigma_pitch_rad * options_.sigma_pitch_rad, options_.sigma_yaw_rad * options_.sigma_yaw_rad;
        return R;
    }
    // Yaw-only: return 3×3 with large roll/pitch noise so they're ignored
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 1e6;
    R(2, 2) = options_.sigma_yaw_rad * options_.sigma_yaw_rad;
    return R;
}

// Wrap rotation-vector differences to [-π, π]
Eigen::VectorXd UkfStarTracker::Innovation(const Eigen::VectorXd& z_obs, const Eigen::VectorXd& z_pred) const {
    Eigen::VectorXd diff = z_obs - z_pred;
    for (int i = 0; i < diff.size(); ++i) diff(i) = core::WrapAngleRad(diff(i));
    return diff;
}

}  // namespace falconguide::estimation::ukf
