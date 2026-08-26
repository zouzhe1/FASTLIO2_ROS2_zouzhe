#include <gtest/gtest.h>

#include "map_tools/tile_id.h"

TEST(TileId, UsesFloorAtPositiveAndNegativeBoundaries)
{
  EXPECT_EQ(map_tools::tileForPoint("L1", 0.0, 24.999).x, 0);
  EXPECT_EQ(map_tools::tileForPoint("L1", 25.0, 0.0).x, 1);
  EXPECT_EQ(map_tools::tileForPoint("L1", -0.001, 0.0).x, -1);
  EXPECT_EQ(map_tools::tileForPoint("L1", -25.0, 0.0).x, -1);
  EXPECT_EQ(map_tools::tileForPoint("L1", -25.001, 0.0).x, -2);
}

TEST(TileId, KeepsLevelsSeparateAtTheSameXY)
{
  const auto lower = map_tools::tileForPoint("B1", 12.0, 12.0);
  const auto upper = map_tools::tileForPoint("L2", 12.0, 12.0);
  EXPECT_NE(lower, upper);
  EXPECT_NE(lower.stableKey(), upper.stableKey());
}
