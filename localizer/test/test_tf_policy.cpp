#include <gtest/gtest.h>

#include "localizer/tf_policy.h"

using localizer::OperationalProfile;
using localizer::TfOwnerConfig;
using localizer::validateTfOwnership;

TEST(TfPolicy, MappingRequiresPoseGraphAsOnlyGlobalOwner)
{
  const auto valid = validateTfOwnership(
    OperationalProfile::MAPPING, TfOwnerConfig{true, false, true, false});
  const auto conflict = validateTfOwnership(
    OperationalProfile::MAPPING, TfOwnerConfig{true, true, true, true});

  EXPECT_TRUE(valid.valid);
  EXPECT_EQ(valid.owner, "pgo");
  EXPECT_FALSE(conflict.valid);
}

TEST(TfPolicy, LocalizationRequiresLocalizerAsOnlyGlobalOwner)
{
  const auto valid = validateTfOwnership(
    OperationalProfile::LOCALIZATION, TfOwnerConfig{false, true, false, true});
  const auto missing = validateTfOwnership(
    OperationalProfile::LOCALIZATION, TfOwnerConfig{false, true, false, false});

  EXPECT_TRUE(valid.valid);
  EXPECT_EQ(valid.owner, "localizer");
  EXPECT_FALSE(missing.valid);
}

TEST(TfPolicy, MaintenanceForbidsOnlineGlobalTf)
{
  const auto valid = validateTfOwnership(
    OperationalProfile::MAINTENANCE, TfOwnerConfig{false, false, false, false});
  const auto invalid = validateTfOwnership(
    OperationalProfile::MAINTENANCE, TfOwnerConfig{false, true, false, true});

  EXPECT_TRUE(valid.valid);
  EXPECT_EQ(valid.owner, "none");
  EXPECT_FALSE(invalid.valid);
}

TEST(TfPolicy, ParsesOnlySupportedProfiles)
{
  EXPECT_EQ(localizer::parseOperationalProfile("mapping"), OperationalProfile::MAPPING);
  EXPECT_EQ(
    localizer::parseOperationalProfile("localization"), OperationalProfile::LOCALIZATION);
  EXPECT_EQ(
    localizer::parseOperationalProfile("maintenance"), OperationalProfile::MAINTENANCE);
  EXPECT_THROW(localizer::parseOperationalProfile("mixed"), std::invalid_argument);
}
