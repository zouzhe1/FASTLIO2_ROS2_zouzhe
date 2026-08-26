#include <gtest/gtest.h>

#include <chrono>
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
}  // namespace

TEST(LatestRegistration, KeepsOneInFlightAndOnlyLatestPendingWork)
{
  localizer::LatestRegistration worker;
  worker.submit({1, [] {std::this_thread::sleep_for(std::chrono::milliseconds(40)); return value(1);}});
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  worker.submit({2, [] {return value(2);}});
  worker.submit({3, [] {return value(3);}});
  EXPECT_EQ(worker.replacedCount(), 1U);

  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  auto result = worker.takeLatest();
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
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  auto result = worker.takeLatest();
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->request_id, 11U);
}
