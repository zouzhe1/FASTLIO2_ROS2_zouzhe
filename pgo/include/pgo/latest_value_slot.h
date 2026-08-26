#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <utility>

namespace pgo
{

template<typename T>
class LatestValueSlot
{
public:
  void push(T value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (value_.has_value()) {
      ++replaced_count_;
    }
    value_ = std::move(value);
  }

  std::optional<T> take()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!value_.has_value()) {
      return std::nullopt;
    }
    std::optional<T> result(std::move(value_));
    value_.reset();
    return result;
  }

  std::size_t pendingCount() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return value_.has_value() ? 1U : 0U;
  }

  std::uint64_t replacedCount() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return replaced_count_;
  }

private:
  mutable std::mutex mutex_;
  std::optional<T> value_;
  std::uint64_t replaced_count_{0};
};

}  // namespace pgo
