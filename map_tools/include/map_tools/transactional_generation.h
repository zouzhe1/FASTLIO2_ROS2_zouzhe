#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>

namespace map_tools
{

class TransactionalGeneration
{
public:
  TransactionalGeneration(std::filesystem::path map_root, std::uint64_t generation);
  ~TransactionalGeneration();

  TransactionalGeneration(const TransactionalGeneration &) = delete;
  TransactionalGeneration & operator=(const TransactionalGeneration &) = delete;

  void write(const std::filesystem::path & relative_path, std::string_view bytes);
  void copyFile(const std::filesystem::path & source, const std::filesystem::path & relative_path);
  void sealExistingFile(const std::filesystem::path & relative_path);
  void publish(std::string_view manifest_yaml);
  const std::filesystem::path & stagingPath() const {return staging_path_;}

private:
  std::filesystem::path checkedTarget(const std::filesystem::path & relative_path) const;
  std::filesystem::path root_;
  std::filesystem::path staging_path_;
  std::filesystem::path final_path_;
  std::uint64_t generation_;
  bool published_{false};
};

std::uint64_t readCurrentGeneration(const std::filesystem::path & map_root);

}  // namespace map_tools
