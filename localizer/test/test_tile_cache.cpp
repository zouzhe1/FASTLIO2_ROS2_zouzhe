#include <gtest/gtest.h>

#include <atomic>
#include <thread>

#include "localizer/tile_cache.h"

namespace
{
localizer::TileLoadResult loaded(const map_tools::TileId & id, std::size_t bytes)
{
  auto data = std::make_shared<localizer::TileData>();
  data->id = id;
  data->bytes.resize(bytes, 1);
  data->checksum = "ok";
  return {localizer::TileLoadStatus::OK, data, ""};
}
}  // namespace

TEST(TileCache, EnforcesLruCountAndByteLimits)
{
  localizer::TileCache cache({2, 10}, "L1");
  auto loader = [](const auto & id) {return loaded(id, 5);};
  auto a = cache.get({"L1", 0, 0}, "ok", loader);
  a.data.reset();
  auto b = cache.get({"L1", 1, 0}, "ok", loader);
  b.data.reset();
  EXPECT_EQ(cache.get({"L1", 0, 0}, "ok", loader).status, localizer::TileLoadStatus::OK);
  auto c = cache.get({"L1", 2, 0}, "ok", loader);
  EXPECT_EQ(c.status, localizer::TileLoadStatus::OK);
  EXPECT_LE(cache.stats().tile_count, 2U);
  EXPECT_LE(cache.stats().bytes, 10U);
  EXPECT_FALSE(cache.contains({"L1", 1, 0}));
}

TEST(TileCache, PinsActiveSnapshotAndRejectsWrongLevelOrCorruption)
{
  localizer::TileCache cache({1, 8}, "L1");
  auto held = cache.get({"L1", 0, 0}, "ok", [](const auto & id) {return loaded(id, 8);});
  auto blocked = cache.get({"L1", 1, 0}, "ok", [](const auto & id) {return loaded(id, 8);});
  EXPECT_EQ(blocked.status, localizer::TileLoadStatus::BUDGET_EXCEEDED);
  EXPECT_EQ(cache.get({"L2", 0, 0}, "ok", [](const auto & id) {return loaded(id, 1);}).status,
    localizer::TileLoadStatus::WRONG_LEVEL);
  held.data.reset();
  auto corrupt = cache.get({"L1", 2, 0}, "expected", [](const auto & id) {return loaded(id, 1);});
  EXPECT_EQ(corrupt.status, localizer::TileLoadStatus::CORRUPT);
}

TEST(TileCache, DeduplicatesConcurrentLoads)
{
  localizer::TileCache cache({4, 100}, "L1");
  std::atomic<int> calls{0};
  auto loader = [&](const auto & id) {
      ++calls;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      return loaded(id, 4);
    };
  localizer::TileLoadResult first, second;
  std::thread one([&] {first = cache.get({"L1", 3, 4}, "ok", loader);});
  std::thread two([&] {second = cache.get({"L1", 3, 4}, "ok", loader);});
  one.join();
  two.join();
  EXPECT_EQ(calls, 1);
  EXPECT_EQ(first.status, localizer::TileLoadStatus::OK);
  EXPECT_EQ(second.status, localizer::TileLoadStatus::OK);
}

TEST(TileCache, RemainsBoundedAcrossSyntheticHundredThousandSquareMetres)
{
  localizer::TileCache cache({9, 900}, "L1");
  auto loader = [](const auto & id) {return loaded(id, 100);};
  for (int x = 0; x < 400; ++x) {
    auto item = cache.get({"L1", x, 0}, "ok", loader);
    ASSERT_EQ(item.status, localizer::TileLoadStatus::OK);
    item.data.reset();
  }
  EXPECT_LE(cache.stats().tile_count, 9U);
  EXPECT_LE(cache.stats().bytes, 900U);
}
