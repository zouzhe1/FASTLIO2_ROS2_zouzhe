#pragma once

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>

#include "localizer/registration_backend.h"

namespace localizer
{

struct RegistrationWork
{
  std::uint64_t request_id{0};
  std::function<RegistrationResult()> execute;
};

struct CompletedRegistration
{
  std::uint64_t request_id{0};
  RegistrationResult result;
};

class LatestRegistration
{
public:
  LatestRegistration();
  ~LatestRegistration();
  LatestRegistration(const LatestRegistration &) = delete;
  LatestRegistration & operator=(const LatestRegistration &) = delete;

  void submit(RegistrationWork work);
  std::optional<CompletedRegistration> takeLatest();
  std::uint64_t replacedCount() const;
  std::uint64_t discardedResultCount() const;

private:
  void run();
  mutable std::mutex mutex_;
  std::condition_variable ready_;
  std::optional<RegistrationWork> pending_;
  std::optional<CompletedRegistration> completed_;
  std::thread worker_;
  bool stop_{false};
  std::uint64_t latest_submitted_{0};
  std::uint64_t replaced_count_{0};
  std::uint64_t discarded_count_{0};
};

}  // namespace localizer
