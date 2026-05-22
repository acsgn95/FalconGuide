#pragma once

/**
 * @file time_ordered_buffer.hpp
 * @brief Timestamp-ordered measurement buffer with optional out-of-order insert
 * support.
 */

#include "falconguide/core/measurement_helpers.hpp"
#include "falconguide/core/time_helpers.hpp"

#include <cstddef>
#include <deque>
#include <optional>
#include <utility>
#include <vector>

namespace falconguide::core {

/// @brief Result of inserting a measurement into a TimeOrderedBuffer.
enum class BufferInsertResult {
    Inserted,                  ///< Measurement was inserted.
    InsertedAndDroppedOldest,  ///< Measurement was inserted and the oldest entry
                               ///< was evicted.
    RejectedOutOfOrder,        ///< Measurement was older than the latest entry and
                               ///< reordering is disabled.
    RejectedNotComparable      ///< Measurement timestamp could not be compared with
                               ///< buffered data.
};

/**
 * @brief Bounded deque that keeps measurements sorted by timestamp.
 * @tparam Measurement Measurement type supported by GetTimestamp().
 */
template <typename Measurement>
class TimeOrderedBuffer {
   public:
    /// @brief Constructs a buffer.
    /// @param max_size Maximum number of elements; zero means unbounded.
    /// @param allow_out_of_order_insert Allow insertion before the latest
    /// element.
    explicit TimeOrderedBuffer(std::size_t max_size = 0, bool allow_out_of_order_insert = true)
        : max_size_(max_size), allow_out_of_order_insert_(allow_out_of_order_insert) {}

    /// @brief Returns the number of buffered measurements.
    [[nodiscard]] std::size_t size() const { return measurements_.size(); }
    /// @brief Returns true when the buffer is empty.
    [[nodiscard]] bool empty() const { return measurements_.empty(); }
    /// @brief Returns the configured maximum size; zero means unbounded.
    [[nodiscard]] std::size_t max_size() const { return max_size_; }

    /// @brief Returns the oldest buffered measurement, or nullptr when empty.
    [[nodiscard]] const Measurement *Oldest() const { return measurements_.empty() ? nullptr : &measurements_.front(); }

    /// @brief Returns the newest buffered measurement, or nullptr when empty.
    [[nodiscard]] const Measurement *Latest() const { return measurements_.empty() ? nullptr : &measurements_.back(); }

    /// @brief Inserts a measurement while preserving timestamp order.
    BufferInsertResult Insert(Measurement measurement) {
        const Timestamp &timestamp = GetTimestamp(measurement);

        if (measurements_.empty()) {
            measurements_.push_back(std::move(measurement));
            return BufferInsertResult::Inserted;
        }

        const TimestampOrdering latest_ordering = CompareTimestamps(GetTimestamp(measurements_.back()), timestamp);
        if (latest_ordering == TimestampOrdering::Before || latest_ordering == TimestampOrdering::Equal) {
            measurements_.push_back(std::move(measurement));
            return EnforceMaxSize(BufferInsertResult::Inserted);
        }

        if (latest_ordering == TimestampOrdering::NotComparable) {
            return BufferInsertResult::RejectedNotComparable;
        }

        if (!allow_out_of_order_insert_) {
            return BufferInsertResult::RejectedOutOfOrder;
        }

        for (auto iterator = measurements_.begin(); iterator != measurements_.end(); ++iterator) {
            const TimestampOrdering ordering = CompareTimestamps(timestamp, GetTimestamp(*iterator));
            if (ordering == TimestampOrdering::NotComparable) {
                return BufferInsertResult::RejectedNotComparable;
            }
            if (ordering == TimestampOrdering::Before || ordering == TimestampOrdering::Equal) {
                measurements_.insert(iterator, std::move(measurement));
                return EnforceMaxSize(BufferInsertResult::Inserted);
            }
        }

        measurements_.push_back(std::move(measurement));
        return EnforceMaxSize(BufferInsertResult::Inserted);
    }

    /// @brief Pops measurements up to a target timestamp.
    /// @param timestamp Target timestamp.
    /// @param include_equal Whether measurements equal to @p timestamp are
    /// popped.
    /// @return Measurements removed from the buffer in chronological order.
    [[nodiscard]] std::vector<Measurement> PopUntil(const Timestamp &timestamp, bool include_equal = true) {
        std::vector<Measurement> result;
        while (!measurements_.empty()) {
            const TimestampOrdering ordering = CompareTimestamps(GetTimestamp(measurements_.front()), timestamp);
            if (ordering == TimestampOrdering::NotComparable) {
                break;
            }
            const bool should_pop =
                ordering == TimestampOrdering::Before || (include_equal && ordering == TimestampOrdering::Equal);
            if (!should_pop) {
                break;
            }

            result.push_back(std::move(measurements_.front()));
            measurements_.pop_front();
        }
        return result;
    }

    /// @brief Removes all buffered measurements.
    void Clear() { measurements_.clear(); }

   private:
    BufferInsertResult EnforceMaxSize(BufferInsertResult insert_result) {
        if (max_size_ == 0 || measurements_.size() <= max_size_) {
            return insert_result;
        }
        measurements_.pop_front();
        return BufferInsertResult::InsertedAndDroppedOldest;
    }

    std::size_t max_size_{0};
    bool allow_out_of_order_insert_{true};
    std::deque<Measurement> measurements_;
};

}  // namespace falconguide::core
