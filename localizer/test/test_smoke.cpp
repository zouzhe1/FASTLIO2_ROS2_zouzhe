#include <gtest/gtest.h>

#include <interface/action/relocalize.hpp>
#include <interface/msg/localization_status.hpp>

TEST(LocalizationInterfaces, ExposeHealthStatesAndRecoveryModes)
{
  EXPECT_EQ(interface::msg::LocalizationStatus::UNINITIALIZED, 0U);
  EXPECT_EQ(interface::msg::LocalizationStatus::TRACKING, 1U);
  EXPECT_EQ(interface::msg::LocalizationStatus::LOST, 3U);
  EXPECT_EQ(interface::action::Relocalize::Goal::AUTO, 0U);
  EXPECT_EQ(interface::action::Relocalize::Goal::ROUGH_POSE, 1U);
}
