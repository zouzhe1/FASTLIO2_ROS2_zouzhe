#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>

#include "map_tools/map_manifest.h"
#include "map_tools/transactional_generation.h"
#include "place_recognition/place_index.h"

namespace fs = std::filesystem;
using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

struct PoseLine
{
  std::string patch;
  Eigen::Vector3f translation;
  Eigen::Quaternionf rotation;
};

int main(int argc, char ** argv)
{
  if (argc < 5 || argc > 7) {
    std::cerr << "usage: tile_builder_node MAP_ROOT SOURCE_GENERATION OUTPUT_GENERATION LEVEL_ID "
                 "[VOXEL_M=0.20] [MAX_BUFFER_POINTS=500000]\n";
    return 2;
  }
  try {
    const fs::path map_root = fs::absolute(argv[1]);
    const std::uint64_t source_generation = std::stoull(argv[2]);
    const std::uint64_t output_generation = std::stoull(argv[3]);
    const std::string level_id = argv[4];
    const float voxel = argc >= 6 ? std::stof(argv[5]) : 0.20F;
    const std::size_t max_buffer_points = argc >= 7 ? std::stoull(argv[6]) : 500000U;
    if (level_id.empty() || voxel <= 0.0F || max_buffer_points == 0) {
      throw std::invalid_argument("invalid level, voxel, or point budget");
    }
    const fs::path source = map_root / ("generation-" + std::to_string(source_generation));
    const YAML::Node source_manifest = YAML::LoadFile((source / "manifest.yaml").string());
    if (source_manifest["artifact_type"].as<std::string>() != "keyframe_stream") {
      throw std::runtime_error("source generation is not a keyframe stream");
    }
    std::map<std::string, std::string> patch_checksums;
    for (const auto & patch : source_manifest["patches"]) {
      patch_checksums.emplace(
        patch["file"].as<std::string>(), patch["checksum"].as<std::string>());
    }

    std::ifstream pose_file(source / "poses.txt");
    std::vector<PoseLine> poses;
    PoseLine pose;
    float tx, ty, tz, qw, qx, qy, qz;
    while (pose_file >> pose.patch >> tx >> ty >> tz >> qw >> qx >> qy >> qz) {
      pose.translation = {tx, ty, tz};
      pose.rotation = Eigen::Quaternionf(qw, qx, qy, qz).normalized();
      poses.push_back(pose);
    }
    if (!pose_file.eof() || poses.empty()) throw std::runtime_error("poses.txt is empty or malformed");

    map_tools::TransactionalGeneration output(map_root, output_generation);
    const fs::path work = output.stagingPath() / ".work";
    std::map<map_tools::TileId, Cloud::Ptr> buffers;
    std::map<map_tools::TileId, std::vector<fs::path>> fragments;
    std::size_t buffered_points = 0;
    std::size_t fragment_sequence = 0;
    place_recognition::PlaceIndex place_index({20, 60, 80.0, 3});
    auto flush = [&]() {
      for (auto & entry : buffers) {
        if (entry.second->empty()) continue;
        const fs::path directory = work / entry.first.stableKey();
        fs::create_directories(directory);
        const fs::path file = directory / (std::to_string(fragment_sequence++) + ".pcd");
        if (pcl::io::savePCDFileBinary(file.string(), *entry.second) != 0) {
          throw std::runtime_error("failed to flush tile fragment");
        }
        fragments[entry.first].push_back(file);
        entry.second->clear();
      }
      buffered_points = 0;
    };

    std::uint64_t place_id = 0;
    for (const auto & item : poses) {
      Cloud body, world;
      const fs::path relative_patch = fs::path("patches") / item.patch;
      const auto expected_checksum = patch_checksums.find(relative_patch.generic_string());
      if (expected_checksum == patch_checksums.end() ||
        map_tools::fileChecksum(source / relative_patch) != expected_checksum->second) {
        throw std::runtime_error("patch checksum mismatch: " + item.patch);
      }
      if (pcl::io::loadPCDFile<Point>((source / relative_patch).string(), body) != 0) {
        throw std::runtime_error("cannot load patch: " + item.patch);
      }
      pcl::transformPointCloud(body, world, item.translation, item.rotation);
      std::vector<place_recognition::Point3> descriptor_points;
      descriptor_points.reserve(body.size());
      for (const auto & point : body) descriptor_points.push_back({point.x, point.y, point.z});
      place_recognition::PlaceMetadata place;
      place.id = place_id++;
      place.x = item.translation.x(); place.y = item.translation.y(); place.z = item.translation.z();
      place.yaw = Eigen::AngleAxisf(item.rotation).axis().z() * Eigen::AngleAxisf(item.rotation).angle();
      place.level_id = level_id;
      place.tile_keys = {map_tools::tileForPoint(level_id, place.x, place.y).stableKey()};
      place.session_id = "offline";
      place.timestamp = static_cast<double>(place.id);
      place_index.add(place, descriptor_points);
      for (const auto & point : world) {
        const auto id = map_tools::tileForPoint(level_id, point.x, point.y);
        auto & tile = buffers[id];
        if (!tile) tile.reset(new Cloud);
        tile->push_back(point);
        if (++buffered_points >= max_buffer_points) flush();
      }
    }
    flush();

    map_tools::MapManifest manifest;
    manifest.map_id = source_manifest["map_id"].as<std::string>();
    manifest.generation = output_generation;
    manifest.frame_id = source_manifest["frame_id"].as<std::string>();
    manifest.created_at = "generated-offline";
    manifest.config_hash = "tile-size=25;voxel=" + std::to_string(voxel);
    manifest.keyframe_index = "places.yaml";
    manifest.levels.push_back({level_id, -1000.0, 1000.0});
    for (const auto & tile_fragments : fragments) {
      Cloud::Ptr merged(new Cloud);
      for (const auto & fragment : tile_fragments.second) {
        Cloud part;
        if (pcl::io::loadPCDFile<Point>(fragment.string(), part) != 0) {
          throw std::runtime_error("cannot reload tile fragment");
        }
        *merged += part;
      }
      pcl::VoxelGrid<Point> filter;
      filter.setLeafSize(voxel, voxel, voxel);
      filter.setInputCloud(merged);
      Cloud filtered;
      filter.filter(filtered);
      const auto & id = tile_fragments.first;
      const fs::path relative = fs::path("tiles") /
        (id.level_id + "_" + std::to_string(id.x) + "_" + std::to_string(id.y) + ".pcd");
      fs::create_directories((output.stagingPath() / relative).parent_path());
      if (pcl::io::savePCDFileBinary((output.stagingPath() / relative).string(), filtered) != 0) {
        throw std::runtime_error("cannot save final tile");
      }
      output.sealExistingFile(relative);
      map_tools::TileRecord record;
      record.id = id;
      record.file = relative.generic_string();
      record.min_x = id.x * manifest.tile_size_m;
      record.max_x = record.min_x + manifest.tile_size_m;
      record.min_y = id.y * manifest.tile_size_m;
      record.max_y = record.min_y + manifest.tile_size_m;
      record.point_count = filtered.size();
      record.voxel_size = voxel;
      record.checksum = map_tools::fileChecksum(output.stagingPath() / relative);
      manifest.tiles.push_back(record);
    }
    fs::remove_all(work);
    output.write(manifest.keyframe_index, place_index.serialize());
    manifest.keyframe_index_checksum = map_tools::fileChecksum(
      output.stagingPath() / manifest.keyframe_index);
    const auto validation = map_tools::validateManifest(manifest, output.stagingPath(), "map", true);
    if (!validation.ok) throw std::runtime_error(validation.reason);
    output.publish(map_tools::serializeManifest(manifest));
    std::cout << "published generation " << output_generation << " with "
              << manifest.tiles.size() << " tiles\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
