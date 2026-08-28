#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <thread>

#include "localizer/latest_registration.h"

namespace
{
localizer::RegistrationResult value(int number)
{
  localizer::RegistrationResult result;
  result.reason = std::to_string(number);
  return result;
}

std::optional<localizer::CompletedRegistration> takeWithTimeout(
  localizer::LatestRegistration & worker, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    auto result = worker.takeLatest();
    if (result) return result;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return worker.takeLatest();
}
}  // namespace

TEST(LatestRegistration, KeepsOneInFlightAndOnlyLatestPendingWork)
{
  localizer::LatestRegistration worker;
  std::promise<void> first_started;
  std::promise<void> release_first;
  auto first_started_future = first_started.get_future();
  auto release_first_future = release_first.get_future();
  worker.submit({1, [&] {
      first_started.set_value();
      release_first_future.wait();
      return value(1);
    }});

  if (first_started_future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
    release_first.set_value();
    FAIL() << "registration worker did not start within the timeout";
  }

  worker.submit({2, [] {return value(2);}});
  worker.submit({3, [] {return value(3);}});
  EXPECT_EQ(worker.replacedCount(), 1U);
  release_first.set_value();

  auto result = takeWithTimeout(worker, std::chrono::seconds(1));
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->request_id, 3U);
  EXPECT_EQ(result->result.reason, "3");
  EXPECT_GE(worker.discardedResultCount(), 1U);
}

TEST(LatestRegistration, ReturnsNoObsoleteResult)
{
  localizer::LatestRegistration worker;
  worker.submit({10, [] {return value(10);}});
  worker.submit({11, [] {return value(11);}});
  auto result = takeWithTimeout(worker, std::chrono::seconds(1));
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->request_id, 11U);
}
