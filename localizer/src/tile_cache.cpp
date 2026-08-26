#include "localizer/tile_cache.h"

#include <utility>

namespace localizer
{

TileCache::TileCache(TileCacheLimits limits, std::string active_level)
: limits_(limits), active_level_(std::move(active_level))
{
  if (limits_.max_tiles == 0 || limits_.max_bytes == 0) {
    throw std::invalid_argument("tile cache limits must be positive");
  }
}

void TileCache::touchLocked(std::map<map_tools::TileId, Entry>::iterator entry)
{
  lru_.erase(entry->second.lru);
  lru_.push_front(entry->first);
  entry->second.lru = lru_.begin();
}

bool TileCache::makeRoomLocked(std::size_t incoming_bytes)
{
  if (incoming_bytes > limits_.max_bytes) return false;
  while (entries_.size() + 1 > limits_.max_tiles || stats_.bytes + incoming_bytes > limits_.max_bytes) {
    auto candidate = lru_.rbegin();
    while (candidate != lru_.rend()) {
      auto entry = entries_.find(*candidate);
      if (entry != entries_.end() && entry->second.data.use_count() == 1) break;
      ++candidate;
    }
    if (candidate == lru_.rend()) return false;
    const auto id = *candidate;
    const auto entry = entries_.find(id);
    stats_.bytes -= entry->second.data->sizeBytes();
    lru_.erase(entry->second.lru);
    entries_.erase(entry);
    ++stats_.evictions;
  }
  return true;
}

TileLoadResult TileCache::getImpl(
  const map_tools::TileId & id, const std::string & expected_checksum, const Loader & loader)
{
  std::shared_ptr<LoadState> state;
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (id.level_id != active_level_) {
      return {TileLoadStatus::WRONG_LEVEL, {}, "tile is outside active level"};
    }
    auto cached = entries_.find(id);
    if (cached != entries_.end()) {
      ++stats_.hits;
      touchLocked(cached);
      return {TileLoadStatus::OK, cached->second.data, "cache hit"};
    }
    auto in_flight = loads_.find(id);
    if (in_flight != loads_.end()) {
      ++stats_.deduplicated_loads;
      state = in_flight->second;
      state->ready.wait(lock, [&] {return state->done;});
      return state->result;
    }
    ++stats_.misses;
    state = std::make_shared<LoadState>();
    loads_[id] = state;
  }

  TileLoadResult result;
  try {
    result = loader(id);
  } catch (const std::exception & error) {
    result = {TileLoadStatus::MISSING, {}, error.what()};
  }
  if (result.status == TileLoadStatus::OK) {
    if (!result.data || result.data->id != id || result.data->checksum != expected_checksum) {
      result = {TileLoadStatus::CORRUPT, {}, "tile identity or checksum mismatch"};
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (result.status == TileLoadStatus::OK) {
      const auto bytes = result.data->sizeBytes();
      if (!makeRoomLocked(bytes)) {
        result = {TileLoadStatus::BUDGET_EXCEEDED, {}, "all eviction candidates are pinned"};
      } else {
        lru_.push_front(id);
        entries_.emplace(id, Entry{result.data, lru_.begin()});
        stats_.tile_count = entries_.size();
        stats_.bytes += bytes;
      }
    }
    state->result = result;
    state->done = true;
    loads_.erase(id);
  }
  state->ready.notify_all();
  return result;
}

void TileCache::setActiveLevel(std::string level_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_level_ = std::move(level_id);
  for (auto entry = entries_.begin(); entry != entries_.end();) {
    if (entry->first.level_id != active_level_ && entry->second.data.use_count() == 1) {
      stats_.bytes -= entry->second.data->sizeBytes();
      lru_.erase(entry->second.lru);
      entry = entries_.erase(entry);
      ++stats_.evictions;
    } else {
      ++entry;
    }
  }
  stats_.tile_count = entries_.size();
}

bool TileCache::contains(const map_tools::TileId & id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return entries_.count(id) != 0;
}

TileCacheStats TileCache::stats() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  TileCacheStats copy = stats_;
  copy.tile_count = entries_.size();
  return copy;
}

}  // namespace localizer
