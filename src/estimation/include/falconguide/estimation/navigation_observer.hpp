#pragma once

#include "falconguide/core/navigation_state.hpp"

#include <memory>

namespace falconguide::estimation {

// ── INavigationObserver ───────────────────────────────────────────────────────
//
// Downstream consumers (control loop, telemetry, data recorder, …) implement
// this interface and register with EstimatorPipeline.
//
// OnNavigationState() is called synchronously on the estimator thread after
// every accepted aiding update.  Implementations must be non-blocking — any
// heavy work (file I/O, network serialisation) must be offloaded to an
// internal worker thread.
//
class INavigationObserver {
 public:
  virtual ~INavigationObserver() = default;

  virtual void OnNavigationState(
      std::shared_ptr<const core::NavigationState> state) = 0;

  // Called when the estimator is reset or loses initialisation.
  virtual void OnEstimatorReset() {}
};

}  // namespace falconguide::estimation
