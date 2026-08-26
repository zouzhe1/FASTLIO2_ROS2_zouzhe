#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "map_tools/tile_id.h"

namespace map_tools
{

constexpr std::uint32_t kManifestSchemaVersion = 1;

struct LevelBand
{
  std::string id;
  double z_min{0.0};
  double z_max{0.0};
};

struct TileRecord
{
  TileId id;
  std::string file;
  double min_x{0.0};
  double max_x{0.0};
  double min_y{0.0};
  double max_y{0.0};
  std::uint64_t point_count{0};
  double voxel_size{0.0};
  std::string checksum;
};

struct MapManifest
{
  std::uint32_t schema_version{kManifestSchemaVersion};
  std::string map_id;
  std::uint64_t generation{0};
  std::string frame_id{"map"};
  double tile_size_m{kDefaultTileSizeM};
  std::string created_at;
  std::string config_hash;
  std::string keyframe_index;
  std::vector<LevelBand> levels;
  std::vector<TileRecord> tiles;
};

struct ValidationResult
{
  bool ok{false};
  std::string reason;
};

std::string serializeManifest(const MapManifest & manifest);
MapManifest parseManifest(const std::string & yaml_text);
std::string fileChecksum(const std::filesystem::path & path);
ValidationResult validateManifest(
  const MapManifest & manifest, const std::filesystem::path & root,
  const std::string & expected_frame, bool verify_checksums,
  double level_overlap_tolerance_m = 0.1);

}  // namespace map_tools
