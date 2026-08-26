#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

#include "map_tools/map_manifest.h"

int main(int argc, char ** argv)
{
  if (argc < 2 || argc > 3) {
    std::cerr << "usage: map_validator GENERATION_DIR [EXPECTED_FRAME]\n";
    return 2;
  }
  const std::filesystem::path root(argv[1]);
  std::ifstream input(root / "manifest.yaml");
  std::ostringstream yaml;
  yaml << input.rdbuf();
  if (!input) {
    std::cerr << "manifest.yaml is missing\n";
    return 1;
  }
  try {
    const auto manifest = map_tools::parseManifest(yaml.str());
    const auto result = map_tools::validateManifest(
      manifest, root, argc == 3 ? argv[2] : "map", true);
    if (!result.ok) {
      std::cerr << result.reason << '\n';
      return 1;
    }
    std::cout << "valid map " << manifest.map_id << " generation " << manifest.generation
              << ", " << manifest.tiles.size() << " tiles\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
