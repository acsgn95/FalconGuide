#pragma once

/**
 * @file navigation_system.hpp
 * @brief Top-level factory and owner for the navigation estimation stack.
 */

#include "falconguide/estimation/estimator_pipeline.hpp"
#include "falconguide/estimation/navigation_system_config.hpp"

namespace falconguide::estimation {

// ── NavigationSystem
// ──────────────────────────────────────────────────────────
//
// Top-level factory and owner of the navigation stack.
//
// Creates an EkfEstimator from NavigationSystemConfig, registers the requested
// sensor measurement models, and wraps everything in an EstimatorPipeline.
//
// Typical usage:
//
//   NavigationSystemConfig cfg;
//   cfg.gnss.enabled = true;
//   cfg.barometer.enabled = true;
//   cfg.barometer.altitude_sigma_m = 0.3;
//
//   NavigationSystem nav(cfg);
//   nav.Pipeline().RegisterObserver(&my_control_loop);
//   nav.Pipeline().Start();
//
//   // from sensor threads:
//   nav.Pipeline().Push(imu_measurement);
//   nav.Pipeline().Push(gnss_solution);
//
/// @brief Builds and owns the configured estimator pipeline.
class NavigationSystem {
public:
  /// @brief Constructs the navigation stack from a system configuration.
  explicit NavigationSystem(const NavigationSystemConfig &config);

  NavigationSystem(const NavigationSystem &) = delete;
  NavigationSystem &operator=(const NavigationSystem &) = delete;
  NavigationSystem(NavigationSystem &&) = delete;
  NavigationSystem &operator=(NavigationSystem &&) = delete;

  /// @brief Returns the mutable estimator pipeline.
  [[nodiscard]] EstimatorPipeline &Pipeline();
  /// @brief Returns the estimator pipeline.
  [[nodiscard]] const EstimatorPipeline &Pipeline() const;

private:
  static std::unique_ptr<INavigationEstimator>
  BuildEstimator(const NavigationSystemConfig &config);
  static std::unique_ptr<INavigationEstimator>
  BuildEkfEstimator(const NavigationSystemConfig &config);
  static std::unique_ptr<INavigationEstimator>
  BuildUkfEstimator(const NavigationSystemConfig &config);
  static std::unique_ptr<INavigationEstimator>
  BuildCeresEstimator(const NavigationSystemConfig &config);
  static std::unique_ptr<INavigationEstimator>
  BuildGtsamEstimator(const NavigationSystemConfig &config);

  EstimatorPipeline pipeline_;
};

} // namespace falconguide::estimation
