#include "falconguide/estimation/backends/ukf/measurement_models/ukf_external_odometry.hpp"

#include "falconguide/core/math.hpp"

#include <variant>

namespace falconguide::estimation::ukf {

UkfExternalOdometry::UkfExternalOdometry(UkfExternalOdometryOptions options) : options_(std::move(options)) {}

bool UkfExternalOdometry::CanHandle(const SensorMeasurement& measurement) const {
  return std::holds_alternative<core::ExternalOdometryMeasurement>(measurement);
}

int UkfExternalOdometry::MeasurementDim(const SensorMeasurement& /*measurement*/) const {
  return (options_.use_position ? 3 : 0) +
         (options_.use_velocity  ? 3 : 0) +
         (options_.use_orientation ? 3 : 0);
}

Eigen::VectorXd UkfExternalOdometry::Predict(
    const NominalState& sigma_state,
    const UkfUpdateContext& /*ctx*/) const {

  const int m = (options_.use_position ? 3 : 0) +
                (options_.use_velocity  ? 3 : 0) +
                (options_.use_orientation ? 3 : 0);
  Eigen::VectorXd z(m);
  int row = 0;

  if (options_.use_position) {
    z.segment<3>(row) = sigma_state.position_enu_m.eigen();
    row += 3;
  }
  if (options_.use_velocity) {
    z.segment<3>(row) = sigma_state.velocity_enu_mps.eigen();
    row += 3;
  }
  if (options_.use_orientation) {
    z.segment<3>(row) = core::LogMapSo3(sigma_state.orientation_body_to_enu);
  }
  return z;
}

std::optional<Eigen::VectorXd> UkfExternalOdometry::Observe(
    const SensorMeasurement& measurement,
    const UkfUpdateContext& ctx) const {

  const auto& eo = std::get<core::ExternalOdometryMeasurement>(measurement);
  if (eo.validity != core::MeasurementValidity::Valid || !ctx.ltp) return std::nullopt;

  const Eigen::Matrix3d R_ecef_to_enu = ctx.ltp->ecef_to_enu_rotation();
  const int m = (options_.use_position ? 3 : 0) +
                (options_.use_velocity  ? 3 : 0) +
                (options_.use_orientation ? 3 : 0);
  Eigen::VectorXd z(m);
  int row = 0;

  if (options_.use_position) {
    z.segment<3>(row) = ctx.ltp->EcefToEnu(eo.position_ecef_m).eigen();
    row += 3;
  }
  if (options_.use_velocity) {
    z.segment<3>(row) = R_ecef_to_enu * eo.velocity_ecef_mps.eigen();
    row += 3;
  }
  if (options_.use_orientation) {
    const Eigen::Quaterniond q_obs =
        (Eigen::Quaterniond(R_ecef_to_enu) * eo.orientation_body_to_ecef).normalized();
    z.segment<3>(row) = core::LogMapSo3(q_obs);
  }
  return z;
}

Eigen::MatrixXd UkfExternalOdometry::NoiseCovariance(
    const SensorMeasurement& /*measurement*/,
    const UkfUpdateContext& /*ctx*/) const {

  const int m = (options_.use_position ? 3 : 0) +
                (options_.use_velocity  ? 3 : 0) +
                (options_.use_orientation ? 3 : 0);
  Eigen::MatrixXd R_meas = Eigen::MatrixXd::Zero(m, m);
  int row = 0;

  if (options_.use_position) {
    const double s = options_.position_sigma_m.value_or(1.0);
    R_meas.block<3, 3>(row, row) = Eigen::Matrix3d::Identity() * (s * s);
    row += 3;
  }
  if (options_.use_velocity) {
    const double s = options_.velocity_sigma_mps.value_or(0.1);
    R_meas.block<3, 3>(row, row) = Eigen::Matrix3d::Identity() * (s * s);
    row += 3;
  }
  if (options_.use_orientation) {
    const double s = options_.orientation_sigma_rad.value_or(0.02);
    R_meas.block<3, 3>(row, row) = Eigen::Matrix3d::Identity() * (s * s);
  }
  return R_meas;
}

}  // namespace falconguide::estimation::ukf
