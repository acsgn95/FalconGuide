#pragma once

/**
 * @file estimator_pipeline.hpp
 * @brief Worker-thread pipeline that feeds measurements into an estimator.
 */

#include "falconguide/estimation/estimator_interface.hpp"
#include "falconguide/estimation/measurement_queue.hpp"
#include "falconguide/estimation/navigation_observer.hpp"

#include <atomic>
#include <memory>
#include <thread>
#include <vector>

namespace falconguide::estimation {

// ── EstimatorPipeline
// ─────────────────────────────────────────────────────────
//
// Runs the navigation estimator on a single dedicated thread.
//
// Sensor threads call Push() to enqueue measurements; the pipeline thread
// drains the queue, forwards each measurement to INavigationEstimator, and
// notifies all registered INavigationObserver instances after every accepted
// aiding update.
//
// Lifecycle:
//   1. Construct with an estimator instance.
//   2. Call RegisterObserver() for each downstream consumer.
//   3. Call Start() — launches the worker thread.
//   4. Push() measurements from sensor threads.
//   5. Call Stop() — drains the queue and joins the thread.
//
// All RegisterObserver() calls must happen before Start(). The pipeline does
// not own observer pointers; observers must outlive the pipeline.
//
/// @brief Owns an estimator, a measurement queue, and navigation observers.
class EstimatorPipeline {
   public:
    /// @brief Creates a pipeline around an estimator.
    /// @param estimator Backend estimator instance owned by the pipeline.
    /// @param queue_capacity Maximum number of queued measurements.
    explicit EstimatorPipeline(std::unique_ptr<INavigationEstimator> estimator, std::size_t queue_capacity = 4096);

    /// @brief Stops the pipeline and releases owned resources.
    ~EstimatorPipeline();

    EstimatorPipeline(const EstimatorPipeline &) = delete;
    EstimatorPipeline &operator=(const EstimatorPipeline &) = delete;
    EstimatorPipeline(EstimatorPipeline &&) = delete;
    EstimatorPipeline &operator=(EstimatorPipeline &&) = delete;

    /// @brief Registers a downstream observer.
    /// @note Must be called before Start(); the observer is not owned.
    void RegisterObserver(INavigationObserver *observer);

    /// @brief Pushes a measurement from any sensor thread.
    /// @return False if the queue is full or the pipeline is not running.
    bool Push(SensorMeasurement measurement);

    /// @brief Starts the estimator worker thread.
    void Start();

    /// @brief Signals shutdown, drains the queue, and joins the worker thread.
    void Stop();

    /// @brief Returns true while the worker thread is running.
    [[nodiscard]] bool IsRunning() const;

    /// @brief Resets the estimator and notifies observers.
    /// @note Must only be called when the pipeline is stopped.
    void Reset();

    /// @brief Returns a const reference to the owned estimator.
    [[nodiscard]] const INavigationEstimator &Estimator() const;

   private:
    void RunLoop();
    void NotifyObservers(std::shared_ptr<const core::NavigationState> state);
    void NotifyReset();

    std::unique_ptr<INavigationEstimator> estimator_;
    MeasurementQueue queue_;
    std::vector<INavigationObserver *> observers_;
    std::thread thread_;
    std::atomic<bool> running_{false};
};

}  // namespace falconguide::estimation
