#include <gtest/gtest.h>

#include "pgo/loop_verifier.h"

TEST(LoopVerifier, RejectsFalseAndOutlierLoopsAndBlacklistsRepeatedPairs)
{
  pgo::LoopVerifier verifier({0.15, 0.15, 5.0, 45.0, 3, 60.0});
  pgo::LoopEvidence bad{1, 9, 0.1, 0.15, false, 0.1, 0.5, 2.0};
  EXPECT_FALSE(verifier.verify(bad, 0.0).accepted);
  EXPECT_FALSE(verifier.verify(bad, 1.0).accepted);
  EXPECT_FALSE(verifier.verify(bad, 2.0).accepted);
  EXPECT_EQ(verifier.verify(bad, 3.0).reason, "blacklisted");

  pgo::LoopEvidence outlier{2, 10, 0.1, 0.4, true, 0.1, 10.0, 2.0};
  EXPECT_EQ(verifier.verify(outlier, 0.0).reason, "correction_limit");
}

TEST(LoopVerifier, AcceptsOnlyDistinctAndGeometricallyVerifiedEvidence)
{
  pgo::LoopVerifier verifier({0.15, 0.15, 5.0, 45.0, 3, 60.0});
  pgo::LoopEvidence good{3, 20, 0.05, 0.4, true, 0.08, 1.0, 5.0};
  EXPECT_TRUE(verifier.verify(good, 0.0).accepted);
  good.second_descriptor_score = 0.1;
  EXPECT_EQ(verifier.verify(good, 1.0).reason, "ambiguous");
}
