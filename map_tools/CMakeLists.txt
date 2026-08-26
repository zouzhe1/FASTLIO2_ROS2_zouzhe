cmake_minimum_required(VERSION 3.8)
project(map_tools)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(map_manifest src/map_manifest.cpp)
target_include_directories(map_manifest PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(map_manifest yaml-cpp)

install(TARGETS map_manifest EXPORT export_map_tools)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_map_tools HAS_LIBRARY_TARGET)
ament_export_dependencies(yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tile_id test/test_tile_id.cpp)
  target_include_directories(test_tile_id PRIVATE include)
  ament_add_gtest(test_map_manifest test/test_map_manifest.cpp)
  target_link_libraries(test_map_manifest map_manifest)
endif()

ament_package()
