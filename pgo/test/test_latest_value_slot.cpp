#include <gtest/gtest.h>

#include "pgo/latest_value_slot.h"

TEST(LatestValueSlot, ReplacesPendingValueAndCountsIt)
{
  pgo::LatestValueSlot<int> slot;

  slot.push(10);
  slot.push(20);

  EXPECT_EQ(slot.pendingCount(), 1U);
  EXPECT_EQ(slot.replacedCount(), 1U);
  auto value = slot.take();
  ASSERT_TRUE(value.has_value());
  EXPECT_EQ(*value, 20);
}

TEST(LatestValueSlot, TakeEmptiesTheSlot)
{
  pgo::LatestValueSlot<int> slot;
  slot.push(7);

  auto value = slot.take();

  ASSERT_TRUE(value.has_value());
  EXPECT_EQ(*value, 7);
  EXPECT_FALSE(slot.take().has_value());
  EXPECT_EQ(slot.pendingCount(), 0U);
}
