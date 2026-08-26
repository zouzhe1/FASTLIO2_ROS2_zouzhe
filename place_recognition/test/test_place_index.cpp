#include <gtest/gtest.h>

#include <cmath>

#include "place_recognition/place_index.h"

namespace
{
std::vector<place_recognition::Point3> scene(double yaw = 0.0)
{
  std::vector<place_recognition::Point3> points;
  for (int i = 0; i < 80; ++i) {
    const double angle = i * 0.071 + yaw;
    const double radius = 5.0 + (i % 9);
    points.push_back({radius * std::cos(angle), radius * std::sin(angle), (i % 7) * 0.4});
  }
  return points;
}
place_recognition::PlaceMetadata metadata(std::uint64_t id, std::string level)
{
  place_recognition::PlaceMetadata value;
  value.id = id; value.level_id = std::move(level); value.session_id = "s1";
  value.tile_keys = {"tile"}; value.timestamp = static_cast<double>(id);
  return value;
}
}  // namespace

TEST(PlaceIndex, OnlineAndOfflineDescriptorsRetrieveTheSamePlace)
{
  place_recognition::PlaceIndex index({20, 60, 80.0, 3});
  index.add(metadata(1, "L1"), scene());
  index.add(metadata(2, "L1"), {{30, 0, 1}, {31, 0, 1}, {32, 0, 1}});
  const auto candidates = index.query(scene(0.15), {"L1", "", 0.0, false, 0, 0, 0});
  ASSERT_FALSE(candidates.empty());
  EXPECT_EQ(candidates.front().metadata.id, 1U);
  EXPECT_LT(candidates.front().score, 0.2);
}

TEST(PlaceIndex, RejectsSameXYWrongLevelAndReportsAmbiguity)
{
  place_recognition::PlaceIndex index({20, 60, 80.0, 3});
  auto lower = metadata(1, "L1"); lower.x = 10; lower.y = 10;
  auto upper = metadata(2, "L2"); upper.x = 10; upper.y = 10;
  index.add(lower, scene()); index.add(upper, scene());
  const auto candidates = index.query(scene(), {"L2", "", 0.0, false, 0, 0, 0});
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates[0].metadata.id, 2U);
}

TEST(PlaceIndex, SerializesVersionAndRejectsConfigurationMismatch)
{
  place_recognition::PlaceIndex index({20, 60, 80.0, 3});
  index.add(metadata(7, "L1"), scene());
  const std::string data = index.serialize();
  auto restored = place_recognition::PlaceIndex::deserialize(data, {20, 60, 80.0, 3});
  EXPECT_EQ(restored.size(), 1U);
  EXPECT_THROW(place_recognition::PlaceIndex::deserialize(data, {10, 30, 40.0, 3}),
    std::runtime_error);
}
