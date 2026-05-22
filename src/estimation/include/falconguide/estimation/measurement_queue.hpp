#pragma once

/**
 * @file measurement_queue.hpp
 * @brief Thread-safe bounded queue for sensor measurement handoff.
 */

#include "falconguide/estimation/estimator_interface.hpp"

#include <condition_variable>
#include <deque>
#include <mutex>
#include <optional>

namespace falconguide::estimation {

// ── MeasurementQueue
// ──────────────────────────────────────────────────────────
//
// Thread-safe bounded MPSC queue that decouples sensor threads from the
// single estimator thread.
//
// Push() is non-blocking and returns false when the queue is full or shutting
// down.  Pop() blocks on the consumer thread until an item arrives or
// RequestShutdown() is called.
//
/// @brief Bounded blocking queue for one consumer and many sensor producers.
class MeasurementQueue {
   public:
    /// @brief Creates a queue with a fixed capacity.
    explicit MeasurementQueue(std::size_t capacity = 4096) : capacity_(capacity) {}

    /// @brief Pushes from any sensor thread.
    /// @return False if the queue is full or shutting down.
    bool Push(SensorMeasurement measurement) {
        std::unique_lock lock(mutex_);
        if (shutdown_ || queue_.size() >= capacity_) return false;
        queue_.push_back(std::move(measurement));
        cv_.notify_one();
        return true;
    }

    /// @brief Pops on the consumer thread, blocking until data or shutdown.
    /// @return Next measurement, or std::nullopt when the pipeline should exit.
    std::optional<SensorMeasurement> Pop() {
        std::unique_lock lock(mutex_);
        cv_.wait(lock, [this] { return !queue_.empty() || shutdown_; });
        if (queue_.empty()) return std::nullopt;
        auto m = std::move(queue_.front());
        queue_.pop_front();
        return m;
    }

    /// @brief Signals shutdown and wakes all waiting consumers.
    void RequestShutdown() {
        std::unique_lock lock(mutex_);
        shutdown_ = true;
        cv_.notify_all();
    }

    /// @brief Returns the current queued item count.
    std::size_t Size() const {
        std::unique_lock lock(mutex_);
        return queue_.size();
    }

   private:
    const std::size_t capacity_;
    std::deque<SensorMeasurement> queue_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    bool shutdown_{false};
};

}  // namespace falconguide::estimation
