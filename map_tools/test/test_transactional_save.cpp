#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "map_tools/transactional_generation.h"

namespace fs = std::filesystem;

TEST(TransactionalGeneration, PublishesManifestLastAndAdvancesCurrentPointer)
{
  const fs::path root = fs::temp_directory_path() / "fastlio_transaction_commit";
  fs::remove_all(root);
  map_tools::TransactionalGeneration save(root, 7);
  save.write("patches/0.pcd", "points");
  save.publish("schema_version: 1\n");

  EXPECT_TRUE(fs::is_regular_file(root / "generation-7/patches/0.pcd"));
  EXPECT_TRUE(fs::is_regular_file(root / "generation-7/manifest.yaml"));
  EXPECT_EQ(map_tools::readCurrentGeneration(root), 7U);
  fs::remove_all(root);
}

TEST(TransactionalGeneration, InterruptionPreservesPreviousGeneration)
{
  const fs::path root = fs::temp_directory_path() / "fastlio_transaction_interrupt";
  fs::remove_all(root);
  {
    map_tools::TransactionalGeneration first(root, 1);
    first.write("patches/0.pcd", "old");
    first.publish("valid: true\n");
  }
  {
    map_tools::TransactionalGeneration interrupted(root, 2);
    interrupted.write("patches/0.pcd", "partial");
  }

  EXPECT_EQ(map_tools::readCurrentGeneration(root), 1U);
  EXPECT_TRUE(fs::is_regular_file(root / "generation-1/manifest.yaml"));
  EXPECT_FALSE(fs::exists(root / "generation-2"));
  fs::remove_all(root);
}

TEST(TransactionalGeneration, RejectsTraversalAndRepeatedGeneration)
{
  const fs::path root = fs::temp_directory_path() / "fastlio_transaction_reject";
  fs::remove_all(root);
  map_tools::TransactionalGeneration save(root, 4);
  EXPECT_THROW(save.write("../escape", "bad"), std::invalid_argument);
  save.publish("valid: true\n");
  EXPECT_THROW(map_tools::TransactionalGeneration(root, 4), std::runtime_error);
  fs::remove_all(root);
}
