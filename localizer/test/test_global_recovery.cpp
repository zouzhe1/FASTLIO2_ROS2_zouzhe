#include <gtest/gtest.h>

#include "localizer/global_recovery.h"

TEST(GlobalRecovery, SelectsVerifiedTopTwoAndRejectsAmbiguityWrongLevelOrDeadline)
{
  localizer::GlobalRecovery recovery({3, 3000.0, 0.15, 3});
  std::vector<localizer::RecoveryCandidate> candidates = {
    {1, "L1", 0.05, false, 1.0}, {2, "L1", 0.10, true, 0.20}, {3, "L1", 0.40, true, 0.60}};
  auto selected = recovery.select(candidates, "L1", 100.0, false);
  EXPECT_TRUE(selected.accepted);
  EXPECT_EQ(selected.candidate_id, 2U);

  candidates[2].geometric_score = 0.30;
  EXPECT_EQ(recovery.select(candidates, "L1", 100.0, false).reason, "ambiguous");
  EXPECT_EQ(recovery.select(candidates, "L2", 100.0, false).reason, "no_verified_candidate");
  EXPECT_EQ(recovery.select(candidates, "L1", 4000.0, false).reason, "deadline_exceeded");
  EXPECT_EQ(recovery.select(candidates, "L1", 100.0, true).reason, "cancelled");
}

TEST(GlobalRecovery, RequiresThreeMotionConsistentFrames)
{
  localizer::GlobalRecovery recovery({3, 3000.0, 0.15, 3});
  EXPECT_FALSE(recovery.confirmMotionFrame(true));
  EXPECT_FALSE(recovery.confirmMotionFrame(false));
  EXPECT_FALSE(recovery.confirmMotionFrame(true));
  EXPECT_FALSE(recovery.confirmMotionFrame(true));
  EXPECT_TRUE(recovery.confirmMotionFrame(true));
}
