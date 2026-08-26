#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "map_tools/map_manifest.h"

namespace fs = std::filesystem;

namespace
{
map_tools::MapManifest validManifest()
{
  map_tools::MapManifest manifest;
  manifest.map_id = "campus";
  manifest.generation = 3;
  manifest.frame_id = "map";
  manifest.created_at = "2026-08-26T00:00:00Z";
  manifest.config_hash = "cfg";
  manifest.keyframe_index = "places.bin";
  manifest.keyframe_index_checksum = "idx";
  manifest.levels.push_back({"L1", -1.0, 3.0});
  map_tools::TileRecord tile;
  tile.id = {"L1", -1, 2};
  tile.file = "tiles/L1_-1_2.pcd";
  tile.min_x = -25.0;
  tile.max_x = 0.0;
  tile.min_y = 50.0;
  tile.max_y = 75.0;
  tile.point_count = 42;
  tile.voxel_size = 0.2;
  tile.checksum = "abc";
  manifest.tiles.push_back(tile);
  return manifest;
}
}  // namespace

TEST(MapManifest, RoundTripsAllRequiredMetadata)
{
  const auto original = validManifest();
  const auto parsed = map_tools::parseManifest(map_tools::serializeManifest(original));
  EXPECT_EQ(parsed.schema_version, map_tools::kManifestSchemaVersion);
  EXPECT_EQ(parsed.map_id, "campus");
  EXPECT_EQ(parsed.generation, 3U);
  ASSERT_EQ(parsed.levels.size(), 1U);
  ASSERT_EQ(parsed.tiles.size(), 1U);
  EXPECT_EQ(parsed.tiles[0].id, original.tiles[0].id);
  EXPECT_EQ(parsed.tiles[0].point_count, 42U);
}

TEST(MapManifest, RejectsUnsupportedSchemaDuplicateTilesAndOverlappingLevels)
{
  auto manifest = validManifest();
  manifest.schema_version = 99;
  EXPECT_FALSE(map_tools::validateManifest(manifest, {}, "map", false).ok);

  manifest = validManifest();
  manifest.tiles.push_back(manifest.tiles.front());
  EXPECT_FALSE(map_tools::validateManifest(manifest, {}, "map", false).ok);

  manifest = validManifest();
  manifest.levels.push_back({"L2", 2.5, 6.0});
  EXPECT_FALSE(map_tools::validateManifest(manifest, {}, "map", false).ok);
}

TEST(MapManifest, RejectsMissingFileAndChecksumMismatchBeforePointLoading)
{
  const fs::path root = fs::temp_directory_path() / "fastlio_manifest_test";
  fs::remove_all(root);
  fs::create_directories(root / "tiles");
  auto manifest = validManifest();
  EXPECT_FALSE(map_tools::validateManifest(manifest, root, "map", true).ok);

  std::ofstream(root / manifest.tiles[0].file) << "tile-data";
  std::ofstream(root / manifest.keyframe_index) << "place-index";
  manifest.tiles[0].checksum = "wrong";
  EXPECT_FALSE(map_tools::validateManifest(manifest, root, "map", true).ok);
  manifest.tiles[0].checksum = map_tools::fileChecksum(root / manifest.tiles[0].file);
  manifest.keyframe_index_checksum = map_tools::fileChecksum(root / manifest.keyframe_index);
  EXPECT_TRUE(map_tools::validateManifest(manifest, root, "map", true).ok);
  std::ofstream(root / manifest.keyframe_index, std::ios::app) << "corrupt";
  EXPECT_FALSE(map_tools::validateManifest(manifest, root, "map", true).ok);
  fs::remove_all(root);
}
