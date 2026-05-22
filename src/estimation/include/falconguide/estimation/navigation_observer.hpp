#pragma once

/**
 * @file navigation_observer.hpp
 * @brief Observer interface for downstream consumers of navigation states.
 */

#include "falconguide/core/navigation_state.hpp"

#include <memory>

namespace falconguide::estimation {

// ── INavigationObserver
// ───────────────────────────────────────────────────────
//
// Downstream consumers (control loop, telemetry, data recorder, …) implement
// this interface and register with EstimatorPipeline.
//
// OnNavigationState() is called synchronously on the estimator thread after
// every accepted aiding update.  Implementations must be non-blocking — any
// heavy work (file I/O, network serialisation) must be offloaded to an
// internal worker thread.
//
/// @brief Observer notified synchronously by EstimatorPipeline.
class INavigationObserver {
public:
  /// @brief Virtual destructor for interface use.
  virtual ~INavigationObserver() = default;

  /// @brief Called after the pipeline publishes a new navigation state.
  /// @param state Shared immutable state snapshot.
  virtual void
  OnNavigationState(std::shared_ptr<const core::NavigationState> state) = 0;

  /// @brief Called when the estimator is reset or loses initialization.
  virtual void OnEstimatorReset() {}
};

} // namespace falconguide::estimation
