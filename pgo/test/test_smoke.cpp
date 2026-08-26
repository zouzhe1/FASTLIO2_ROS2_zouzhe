#include <gtest/gtest.h>

#include <interface/action/relocalize.hpp>
#include <interface/msg/localization_status.hpp>

TEST(LocalizationInterfaces, AreSharedWithPoseGraph)
{
  interface::msg::LocalizationStatus status;
  status.operational_profile = "mapping";
  status.tf_owner = "pgo";
  status.global_tf_published = true;

  EXPECT_EQ(status.operational_profile, "mapping");
  EXPECT_EQ(status.tf_owner, "pgo");
  EXPECT_TRUE(status.global_tf_published);
}
