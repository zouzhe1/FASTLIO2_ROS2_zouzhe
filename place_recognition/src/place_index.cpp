#include "place_recognition/place_index.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace place_recognition
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
bool sameConfig(const PlaceIndexConfig & a, const PlaceIndexConfig & b)
{
  return a.rings == b.rings && a.sectors == b.sectors &&
    std::abs(a.max_radius - b.max_radius) < 1e-9 && a.top_k == b.top_k;
}
}  // namespace

PlaceIndex::PlaceIndex(PlaceIndexConfig config) : config_(config)
{
  if (config_.rings == 0 || config_.sectors == 0 || config_.max_radius <= 0 || config_.top_k == 0) {
    throw std::invalid_argument("invalid place index bounds");
  }
}

std::vector<float> PlaceIndex::describe(const std::vector<Point3> & points) const
{
  std::vector<float> descriptor(config_.rings * config_.sectors, 0.0F);
  for (const auto & point : points) {
    const double radius = std::hypot(point.x, point.y);
    if (!std::isfinite(radius) || !std::isfinite(point.z) || radius >= config_.max_radius) continue;
    double angle = std::atan2(point.y, point.x);
    if (angle < 0) angle += 2.0 * kPi;
    const auto ring = std::min(config_.rings - 1,
      static_cast<std::size_t>(radius / config_.max_radius * config_.rings));
    const auto sector = std::min(config_.sectors - 1,
      static_cast<std::size_t>(angle / (2.0 * kPi) * config_.sectors));
    auto & bin = descriptor[ring * config_.sectors + sector];
    bin = std::max(bin, static_cast<float>(point.z + 2.0));
  }
  return descriptor;
}

std::pair<double, std::size_t> PlaceIndex::distance(
  const std::vector<float> & query, const std::vector<float> & target) const
{
  double best = std::numeric_limits<double>::infinity();
  std::size_t best_shift = 0;
  for (std::size_t shift = 0; shift < config_.sectors; ++shift) {
    double dot = 0, query_norm = 0, target_norm = 0;
    for (std::size_t ring = 0; ring < config_.rings; ++ring) {
      for (std::size_t sector = 0; sector < config_.sectors; ++sector) {
        const double a = query[ring * config_.sectors + sector];
        const double b = target[ring * config_.sectors + (sector + shift) % config_.sectors];
        dot += a * b; query_norm += a * a; target_norm += b * b;
      }
    }
    const double similarity = dot / std::sqrt(std::max(1e-12, query_norm * target_norm));
    const double score = 1.0 - similarity;
    if (score < best) {best = score; best_shift = shift;}
  }
  return {best, best_shift};
}

void PlaceIndex::add(const PlaceMetadata & metadata, const std::vector<Point3> & points)
{
  if (metadata.level_id.empty() || metadata.tile_keys.empty() || points.empty()) {
    throw std::invalid_argument("place metadata and points must be complete");
  }
  entries_.push_back({metadata, describe(points)});
}

std::vector<PlaceCandidate> PlaceIndex::query(
  const std::vector<Point3> & points, const QueryOptions & options) const
{
  const auto descriptor = describe(points);
  std::vector<PlaceCandidate> candidates;
  for (const auto & entry : entries_) {
    if (!options.level_id.empty() && entry.metadata.level_id != options.level_id) continue;
    if (!options.excluded_session.empty() && entry.metadata.session_id == options.excluded_session &&
      std::abs(entry.metadata.timestamp) < options.minimum_time_separation) continue;
    if (options.use_rough_pose && std::hypot(
      entry.metadata.x - options.rough_x, entry.metadata.y - options.rough_y) > options.rough_radius) continue;
    const auto [score, shift] = distance(descriptor, entry.descriptor);
    candidates.push_back({entry.metadata, score,
      static_cast<double>(shift) * 2.0 * kPi / static_cast<double>(config_.sectors)});
  }
  std::sort(candidates.begin(), candidates.end(), [](const auto & a, const auto & b) {
    return a.score < b.score;
  });
  if (candidates.size() > config_.top_k) candidates.resize(config_.top_k);
  return candidates;
}

std::string PlaceIndex::configHash() const
{
  return "sc-v1-r" + std::to_string(config_.rings) + "-s" + std::to_string(config_.sectors) +
    "-m" + std::to_string(config_.max_radius) + "-k" + std::to_string(config_.top_k);
}

std::string PlaceIndex::serialize() const
{
  YAML::Emitter out;
  out << YAML::BeginMap << YAML::Key << "version" << YAML::Value << kDescriptorVersion
      << YAML::Key << "config_hash" << YAML::Value << configHash()
      << YAML::Key << "rings" << YAML::Value << config_.rings
      << YAML::Key << "sectors" << YAML::Value << config_.sectors
      << YAML::Key << "max_radius" << YAML::Value << config_.max_radius
      << YAML::Key << "top_k" << YAML::Value << config_.top_k
      << YAML::Key << "entries" << YAML::Value << YAML::BeginSeq;
  for (const auto & entry : entries_) {
    out << YAML::BeginMap << YAML::Key << "id" << YAML::Value << entry.metadata.id
        << YAML::Key << "pose" << YAML::Value << YAML::Flow << YAML::BeginSeq
        << entry.metadata.x << entry.metadata.y << entry.metadata.z << entry.metadata.yaw << YAML::EndSeq
        << YAML::Key << "level_id" << YAML::Value << entry.metadata.level_id
        << YAML::Key << "tiles" << YAML::Value << entry.metadata.tile_keys
        << YAML::Key << "session" << YAML::Value << entry.metadata.session_id
        << YAML::Key << "timestamp" << YAML::Value << entry.metadata.timestamp
        << YAML::Key << "checksum" << YAML::Value << entry.metadata.checksum
        << YAML::Key << "descriptor" << YAML::Value << entry.descriptor << YAML::EndMap;
  }
  out << YAML::EndSeq << YAML::EndMap;
  return out.c_str();
}

PlaceIndex PlaceIndex::deserialize(const std::string & data, const PlaceIndexConfig & expected)
{
  const auto root = YAML::Load(data);
  if (root["version"].as<std::uint32_t>() != kDescriptorVersion) {
    throw std::runtime_error("unsupported descriptor version");
  }
  PlaceIndexConfig stored{root["rings"].as<std::size_t>(), root["sectors"].as<std::size_t>(),
    root["max_radius"].as<double>(), root["top_k"].as<std::size_t>()};
  if (!sameConfig(stored, expected)) throw std::runtime_error("descriptor configuration mismatch");
  PlaceIndex index(expected);
  if (root["config_hash"].as<std::string>() != index.configHash()) {
    throw std::runtime_error("descriptor config hash mismatch");
  }
  for (const auto & node : root["entries"]) {
    Entry entry;
    entry.metadata.id = node["id"].as<std::uint64_t>();
    const auto pose = node["pose"];
    entry.metadata.x = pose[0].as<double>(); entry.metadata.y = pose[1].as<double>();
    entry.metadata.z = pose[2].as<double>(); entry.metadata.yaw = pose[3].as<double>();
    entry.metadata.level_id = node["level_id"].as<std::string>();
    entry.metadata.tile_keys = node["tiles"].as<std::vector<std::string>>();
    entry.metadata.session_id = node["session"].as<std::string>();
    entry.metadata.timestamp = node["timestamp"].as<double>();
    entry.metadata.checksum = node["checksum"].as<std::string>();
    entry.descriptor = node["descriptor"].as<std::vector<float>>();
    if (entry.descriptor.size() != expected.rings * expected.sectors) {
      throw std::runtime_error("descriptor size mismatch");
    }
    index.entries_.push_back(std::move(entry));
  }
  return index;
}

}  // namespace place_recognition
