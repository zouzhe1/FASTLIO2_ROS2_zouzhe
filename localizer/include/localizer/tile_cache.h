#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <stdexcept>
#include <utility>
#include <vector>

#include "map_tools/tile_id.h"

namespace localizer
{

enum class TileLoadStatus
{
  OK,
  MISSING,
  CORRUPT,
  WRONG_LEVEL,
  BUDGET_EXCEEDED
};

struct TileData
{
  map_tools::TileId id;
  std::vector<std::uint8_t> bytes;
  std::string checksum;
};

struct TileLoadResult
{
  TileLoadStatus status{TileLoadStatus::MISSING};
  std::shared_ptr<const TileData> data;
  std::string reason;
};

struct TileCacheLimits
{
  std::size_t max_tiles{9};
  std::size_t max_bytes{256U * 1024U * 1024U};
};

struct TileCacheStats
{
  std::size_t tile_count{0};
  std::size_t bytes{0};
  std::uint64_t hits{0};
  std::uint64_t misses{0};
  std::uint64_t evictions{0};
  std::uint64_t deduplicated_loads{0};
};

class TileCache
{
public:
  using Loader = std::function<TileLoadResult(const map_tools::TileId &)>;

  TileCache(TileCacheLimits limits, std::string active_level);

  template<typename LoaderT>
  TileLoadResult get(
    const map_tools::TileId & id, const std::string & expected_checksum, LoaderT && loader)
  {
    return getImpl(id, expected_checksum, Loader(std::forward<LoaderT>(loader)));
  }

  void setActiveLevel(std::string level_id);
  bool contains(const map_tools::TileId & id) const;
  TileCacheStats stats() const;

private:
  struct Entry
  {
    std::shared_ptr<const TileData> data;
    std::list<map_tools::TileId>::iterator lru;
  };
  struct LoadState
  {
    std::condition_variable ready;
    bool done{false};
    TileLoadResult result;
  };

  TileLoadResult getImpl(
    const map_tools::TileId & id, const std::string & expected_checksum, const Loader & loader);
  bool makeRoomLocked(std::size_t incoming_bytes);
  void touchLocked(std::map<map_tools::TileId, Entry>::iterator entry);

  TileCacheLimits limits_;
  std::string active_level_;
  mutable std::mutex mutex_;
  std::map<map_tools::TileId, Entry> entries_;
  std::map<map_tools::TileId, std::shared_ptr<LoadState>> loads_;
  std::list<map_tools::TileId> lru_;
  TileCacheStats stats_;
};

}  // namespace localizer
