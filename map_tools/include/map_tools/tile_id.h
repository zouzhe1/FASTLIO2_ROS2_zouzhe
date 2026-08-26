#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <tuple>

namespace map_tools
{

constexpr double kDefaultTileSizeM = 25.0;

struct TileId
{
  std::string level_id;
  std::int64_t x{0};
  std::int64_t y{0};

  std::string stableKey() const
  {
    return level_id + "/" + std::to_string(x) + "/" + std::to_string(y);
  }

  bool operator==(const TileId & other) const
  {
    return std::tie(level_id, x, y) == std::tie(other.level_id, other.x, other.y);
  }
  bool operator!=(const TileId & other) const {return !(*this == other);}
  bool operator<(const TileId & other) const
  {
    return std::tie(level_id, x, y) < std::tie(other.level_id, other.x, other.y);
  }
};

inline TileId tileForPoint(
  const std::string & level_id, double x, double y,
  double tile_size_m = kDefaultTileSizeM)
{
  return {
    level_id,
    static_cast<std::int64_t>(std::floor(x / tile_size_m)),
    static_cast<std::int64_t>(std::floor(y / tile_size_m))};
}

}  // namespace map_tools
