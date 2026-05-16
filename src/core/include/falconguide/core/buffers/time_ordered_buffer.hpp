#pragma once

#include "falconguide/core/measurement_helpers.hpp"
#include "falconguide/core/time_helpers.hpp"

#include <cstddef>
#include <deque>
#include <optional>
#include <utility>
#include <vector>

namespace falconguide::core {

enum class BufferInsertResult {
  Inserted,
  InsertedAndDroppedOldest,
  RejectedOutOfOrder,
  RejectedNotComparable
};

template <typename Measurement>
class TimeOrderedBuffer {
 public:
  explicit TimeOrderedBuffer(std::size_t max_size = 0, bool allow_out_of_order_insert = true)
      : max_size_(max_size), allow_out_of_order_insert_(allow_out_of_order_insert) {}

  [[nodiscard]] std::size_t size() const { return measurements_.size(); }
  [[nodiscard]] bool empty() const { return measurements_.empty(); }
  [[nodiscard]] std::size_t max_size() const { return max_size_; }

  [[nodiscard]] const Measurement* Oldest() const {
    return measurements_.empty() ? nullptr : &measurements_.front();
  }

  [[nodiscard]] const Measurement* Latest() const {
    return measurements_.empty() ? nullptr : &measurements_.back();
  }

  BufferInsertResult Insert(Measurement measurement) {
    const Timestamp& timestamp = GetTimestamp(measurement);

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

  [[nodiscard]] std::vector<Measurement> PopUntil(const Timestamp& timestamp, bool include_equal = true) {
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

  void Clear() {
    measurements_.clear();
  }

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
