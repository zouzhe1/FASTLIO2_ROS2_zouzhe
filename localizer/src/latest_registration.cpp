#include "localizer/latest_registration.h"

#include <stdexcept>
#include <utility>

namespace localizer
{

LatestRegistration::LatestRegistration() : worker_(&LatestRegistration::run, this) {}

LatestRegistration::~LatestRegistration()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
    pending_.reset();
  }
  ready_.notify_all();
  worker_.join();
}

void LatestRegistration::submit(RegistrationWork work)
{
  if (!work.execute) throw std::invalid_argument("registration work callback is empty");
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (work.request_id <= latest_submitted_) {
      throw std::invalid_argument("registration request IDs must increase");
    }
    latest_submitted_ = work.request_id;
    if (pending_) ++replaced_count_;
    pending_ = std::move(work);
  }
  ready_.notify_one();
}

void LatestRegistration::run()
{
  while (true) {
    RegistrationWork work;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      ready_.wait(lock, [&] {return stop_ || pending_.has_value();});
      if (stop_) return;
      work = std::move(*pending_);
      pending_.reset();
    }
    RegistrationResult result;
    try {
      result = work.execute();
    } catch (const std::exception & error) {
      result.reason = std::string("worker_exception:") + error.what();
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (work.request_id == latest_submitted_) {
      completed_ = CompletedRegistration{work.request_id, std::move(result)};
    } else {
      ++discarded_count_;
    }
  }
}

std::optional<CompletedRegistration> LatestRegistration::takeLatest()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!completed_ || completed_->request_id != latest_submitted_) return std::nullopt;
  auto result = std::move(completed_);
  completed_.reset();
  return result;
}

std::uint64_t LatestRegistration::replacedCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return replaced_count_;
}

std::uint64_t LatestRegistration::discardedResultCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return discarded_count_;
}

}  // namespace localizer
