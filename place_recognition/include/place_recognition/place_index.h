#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace place_recognition
{

constexpr std::uint32_t kDescriptorVersion = 1;

struct Point3 {double x{0}; double y{0}; double z{0};};

struct PlaceIndexConfig
{
  std::size_t rings{20};
  std::size_t sectors{60};
  double max_radius{80.0};
  std::size_t top_k{3};
};

struct PlaceMetadata
{
  std::uint64_t id{0};
  double x{0}; double y{0}; double z{0}; double yaw{0};
  std::string level_id;
  std::vector<std::string> tile_keys;
  std::string session_id;
  double timestamp{0};
  std::string checksum;
};

struct QueryOptions
{
  std::string level_id;
  std::string excluded_session;
  double minimum_time_separation{0};
  bool use_rough_pose{false};
  double rough_x{0}; double rough_y{0}; double rough_radius{0};
};

struct PlaceCandidate
{
  PlaceMetadata metadata;
  double score{1.0};
  double yaw_hint{0.0};
};

class PlaceIndex
{
public:
  explicit PlaceIndex(PlaceIndexConfig config);
  void add(const PlaceMetadata & metadata, const std::vector<Point3> & points);
  std::vector<PlaceCandidate> query(
    const std::vector<Point3> & points, const QueryOptions & options) const;
  std::string serialize() const;
  static PlaceIndex deserialize(const std::string & data, const PlaceIndexConfig & expected);
  std::size_t size() const {return entries_.size();}
  std::string configHash() const;

private:
  struct Entry {PlaceMetadata metadata; std::vector<float> descriptor;};
  std::vector<float> describe(const std::vector<Point3> & points) const;
  std::pair<double, std::size_t> distance(
    const std::vector<float> & query, const std::vector<float> & target) const;
  PlaceIndexConfig config_;
  std::vector<Entry> entries_;
};

}  // namespace place_recognition
