#include "map_tools/transactional_generation.h"

#include <fstream>
#include <stdexcept>

#ifndef _WIN32
#include <fcntl.h>
#include <unistd.h>
#endif

namespace map_tools
{
namespace
{
void syncFile(const std::filesystem::path & path)
{
#ifndef _WIN32
  const int fd = ::open(path.c_str(), O_RDONLY);
  if (fd < 0 || ::fsync(fd) != 0) {
    if (fd >= 0) ::close(fd);
    throw std::runtime_error("fsync failed: " + path.string());
  }
  ::close(fd);
#else
  (void)path;
#endif
}
}  // namespace

TransactionalGeneration::TransactionalGeneration(
  std::filesystem::path map_root, std::uint64_t generation)
: root_(std::filesystem::absolute(std::move(map_root)).lexically_normal()),
  staging_path_(root_ / (".generation-" + std::to_string(generation) + ".tmp")),
  final_path_(root_ / ("generation-" + std::to_string(generation))), generation_(generation)
{
  std::filesystem::create_directories(root_);
  if (std::filesystem::exists(final_path_) || std::filesystem::exists(staging_path_)) {
    throw std::runtime_error("map generation already exists");
  }
  std::filesystem::create_directory(staging_path_);
}

TransactionalGeneration::~TransactionalGeneration()
{
  if (!published_) {
    std::error_code error;
    std::filesystem::remove_all(staging_path_, error);
  }
}

std::filesystem::path TransactionalGeneration::checkedTarget(
  const std::filesystem::path & relative_path) const
{
  if (relative_path.empty() || relative_path.is_absolute()) {
    throw std::invalid_argument("generation file path must be relative");
  }
  for (const auto & component : relative_path) {
    if (component == "..") throw std::invalid_argument("generation file path escapes root");
  }
  const auto target = (staging_path_ / relative_path).lexically_normal();
  const auto relative = target.lexically_relative(staging_path_);
  if (relative.empty() || *relative.begin() == "..") {
    throw std::invalid_argument("generation file path escapes root");
  }
  return target;
}

void TransactionalGeneration::write(
  const std::filesystem::path & relative_path, std::string_view bytes)
{
  if (published_) throw std::logic_error("generation is already published");
  const auto target = checkedTarget(relative_path);
  std::filesystem::create_directories(target.parent_path());
  std::ofstream output(target, std::ios::binary | std::ios::trunc);
  output.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  output.flush();
  if (!output) throw std::runtime_error("failed to write generation file: " + target.string());
  output.close();
  syncFile(target);
}

void TransactionalGeneration::copyFile(
  const std::filesystem::path & source, const std::filesystem::path & relative_path)
{
  if (published_) throw std::logic_error("generation is already published");
  const auto target = checkedTarget(relative_path);
  std::filesystem::create_directories(target.parent_path());
  std::filesystem::copy_file(source, target, std::filesystem::copy_options::none);
  syncFile(target);
}

void TransactionalGeneration::sealExistingFile(const std::filesystem::path & relative_path)
{
  if (published_) throw std::logic_error("generation is already published");
  const auto target = checkedTarget(relative_path);
  if (!std::filesystem::is_regular_file(target)) {
    throw std::runtime_error("generation file is missing: " + target.string());
  }
  syncFile(target);
}

void TransactionalGeneration::publish(std::string_view manifest_yaml)
{
  write("manifest.yaml", manifest_yaml);
  std::filesystem::rename(staging_path_, final_path_);

  const auto next_pointer = root_ / ".current.tmp";
  {
    std::ofstream output(next_pointer, std::ios::trunc);
    output << generation_ << '\n';
    output.flush();
    if (!output) throw std::runtime_error("failed to write current generation pointer");
  }
  syncFile(next_pointer);
  std::filesystem::rename(next_pointer, root_ / "current");
  published_ = true;
}

std::uint64_t readCurrentGeneration(const std::filesystem::path & map_root)
{
  std::ifstream input(map_root / "current");
  std::uint64_t generation = 0;
  if (!(input >> generation)) throw std::runtime_error("no valid current map generation");
  return generation;
}

}  // namespace map_tools
