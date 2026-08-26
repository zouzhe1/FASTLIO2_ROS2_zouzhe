#include "map_tools/map_manifest.h"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace map_tools
{

std::string serializeManifest(const MapManifest & manifest)
{
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "schema_version" << YAML::Value << manifest.schema_version;
  out << YAML::Key << "map_id" << YAML::Value << manifest.map_id;
  out << YAML::Key << "generation" << YAML::Value << manifest.generation;
  out << YAML::Key << "frame_id" << YAML::Value << manifest.frame_id;
  out << YAML::Key << "tile_size_m" << YAML::Value << manifest.tile_size_m;
  out << YAML::Key << "created_at" << YAML::Value << manifest.created_at;
  out << YAML::Key << "config_hash" << YAML::Value << manifest.config_hash;
  out << YAML::Key << "keyframe_index" << YAML::Value << manifest.keyframe_index;
  out << YAML::Key << "keyframe_index_checksum" << YAML::Value << manifest.keyframe_index_checksum;
  out << YAML::Key << "levels" << YAML::Value << YAML::BeginSeq;
  for (const auto & level : manifest.levels) {
    out << YAML::BeginMap << YAML::Key << "id" << YAML::Value << level.id
        << YAML::Key << "z_min" << YAML::Value << level.z_min
        << YAML::Key << "z_max" << YAML::Value << level.z_max << YAML::EndMap;
  }
  out << YAML::EndSeq;
  out << YAML::Key << "tiles" << YAML::Value << YAML::BeginSeq;
  for (const auto & tile : manifest.tiles) {
    out << YAML::BeginMap
        << YAML::Key << "level_id" << YAML::Value << tile.id.level_id
        << YAML::Key << "x" << YAML::Value << tile.id.x
        << YAML::Key << "y" << YAML::Value << tile.id.y
        << YAML::Key << "file" << YAML::Value << tile.file
        << YAML::Key << "bounds" << YAML::Value << YAML::Flow << YAML::BeginSeq
        << tile.min_x << tile.max_x << tile.min_y << tile.max_y << YAML::EndSeq
        << YAML::Key << "point_count" << YAML::Value << tile.point_count
        << YAML::Key << "voxel_size" << YAML::Value << tile.voxel_size
        << YAML::Key << "checksum" << YAML::Value << tile.checksum << YAML::EndMap;
  }
  out << YAML::EndSeq << YAML::EndMap;
  if (!out.good()) {
    throw std::runtime_error("failed to serialize map manifest");
  }
  return out.c_str();
}

MapManifest parseManifest(const std::string & yaml_text)
{
  const YAML::Node root = YAML::Load(yaml_text);
  MapManifest manifest;
  manifest.schema_version = root["schema_version"].as<std::uint32_t>();
  manifest.map_id = root["map_id"].as<std::string>();
  manifest.generation = root["generation"].as<std::uint64_t>();
  manifest.frame_id = root["frame_id"].as<std::string>();
  manifest.tile_size_m = root["tile_size_m"].as<double>();
  manifest.created_at = root["created_at"].as<std::string>();
  manifest.config_hash = root["config_hash"].as<std::string>();
  manifest.keyframe_index = root["keyframe_index"].as<std::string>();
  manifest.keyframe_index_checksum = root["keyframe_index_checksum"].as<std::string>();
  for (const auto & node : root["levels"]) {
    manifest.levels.push_back({
      node["id"].as<std::string>(), node["z_min"].as<double>(), node["z_max"].as<double>()});
  }
  for (const auto & node : root["tiles"]) {
    TileRecord tile;
    tile.id = {node["level_id"].as<std::string>(), node["x"].as<std::int64_t>(),
      node["y"].as<std::int64_t>()};
    tile.file = node["file"].as<std::string>();
    const auto bounds = node["bounds"];
    if (!bounds.IsSequence() || bounds.size() != 4) {
      throw std::runtime_error("tile bounds must contain four values");
    }
    tile.min_x = bounds[0].as<double>();
    tile.max_x = bounds[1].as<double>();
    tile.min_y = bounds[2].as<double>();
    tile.max_y = bounds[3].as<double>();
    tile.point_count = node["point_count"].as<std::uint64_t>();
    tile.voxel_size = node["voxel_size"].as<double>();
    tile.checksum = node["checksum"].as<std::string>();
    manifest.tiles.push_back(std::move(tile));
  }
  return manifest;
}

std::string fileChecksum(const std::filesystem::path & path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("cannot open file for checksum: " + path.string());
  }
  std::uint64_t hash = 14695981039346656037ULL;
  char buffer[8192];
  while (input.read(buffer, sizeof(buffer)) || input.gcount() > 0) {
    for (std::streamsize i = 0; i < input.gcount(); ++i) {
      hash ^= static_cast<unsigned char>(buffer[i]);
      hash *= 1099511628211ULL;
    }
  }
  std::ostringstream text;
  text << "fnv1a64:" << std::hex << std::setw(16) << std::setfill('0') << hash;
  return text.str();
}

ValidationResult validateManifest(
  const MapManifest & manifest, const std::filesystem::path & root,
  const std::string & expected_frame, bool verify_checksums,
  double level_overlap_tolerance_m)
{
  auto fail = [](const std::string & reason) {return ValidationResult{false, reason};};
  if (manifest.schema_version != kManifestSchemaVersion) return fail("unsupported schema version");
  if (manifest.map_id.empty()) return fail("map_id is required");
  if (manifest.frame_id != expected_frame) return fail("frame mismatch");
  if (manifest.tile_size_m <= 0.0) return fail("invalid tile size");
  if (manifest.keyframe_index.empty()) return fail("keyframe index is required");
  if (manifest.keyframe_index_checksum.empty()) return fail("keyframe index checksum is required");
  const auto safe_relative = [](const std::filesystem::path & path) {
    if (path.empty() || path.is_absolute()) return false;
    return std::none_of(path.begin(), path.end(), [](const auto & part) {return part == "..";});
  };
  if (!safe_relative(manifest.keyframe_index)) return fail("unsafe keyframe index path");
  if (!root.empty() && !std::filesystem::is_regular_file(root / manifest.keyframe_index)) {
    return fail("missing keyframe index");
  }
  if (!root.empty() && verify_checksums &&
    fileChecksum(root / manifest.keyframe_index) != manifest.keyframe_index_checksum) {
    return fail("keyframe index checksum mismatch");
  }
  std::set<std::string> level_ids;
  for (const auto & level : manifest.levels) {
    if (level.id.empty() || level.z_min >= level.z_max || !level_ids.insert(level.id).second) {
      return fail("invalid or duplicate level");
    }
  }
  for (size_t i = 0; i < manifest.levels.size(); ++i) {
    for (size_t j = i + 1; j < manifest.levels.size(); ++j) {
      const double overlap = std::min(manifest.levels[i].z_max, manifest.levels[j].z_max) -
        std::max(manifest.levels[i].z_min, manifest.levels[j].z_min);
      if (overlap > level_overlap_tolerance_m) return fail("level bands overlap");
    }
  }
  std::set<TileId> ids;
  for (const auto & tile : manifest.tiles) {
    if (!ids.insert(tile.id).second) return fail("duplicate tile id");
    if (level_ids.count(tile.id.level_id) == 0) return fail("tile references unknown level");
    if (!safe_relative(tile.file) || tile.checksum.empty() || tile.point_count == 0 || tile.voxel_size <= 0.0) {
      return fail("tile metadata incomplete");
    }
    if (!root.empty()) {
      const auto file = root / tile.file;
      if (!std::filesystem::is_regular_file(file)) return fail("missing tile file: " + tile.file);
      if (verify_checksums && fileChecksum(file) != tile.checksum) {
        return fail("tile checksum mismatch: " + tile.file);
      }
    }
  }
  return {true, "ok"};
}

}  // namespace map_tools
