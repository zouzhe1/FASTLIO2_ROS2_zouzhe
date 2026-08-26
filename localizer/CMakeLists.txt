cmake_minimum_required(VERSION 3.8)
project(localizer)

option(LOCALIZER_BUILD_NODE "Build the ROS localizer node" ON)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(interface REQUIRED)
find_package(map_tools REQUIRED)
find_package(small_gicp_vendor REQUIRED)
find_package(small_gicp REQUIRED)

add_library(localization_core
  src/localization_state_machine.cpp
  src/tf_policy.cpp
  src/tile_cache.cpp
  src/small_gicp_backend.cpp
  src/registration_quality_gate.cpp
  src/latest_registration.cpp
)
target_include_directories(localization_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(localization_core map_tools)
target_link_libraries(localization_core small_gicp::small_gicp)

if(LOCALIZER_BUILD_NODE)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)



include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


set(SRC_LIST src/localizers/commons.cpp
             src/localizers/icp_localizer.cpp)

add_executable(localizer_node src/localizer_node.cpp ${SRC_LIST})
ament_target_dependencies(localizer_node rclcpp std_msgs sensor_msgs nav_msgs message_filters pcl_conversions tf2_ros geometry_msgs interface)
target_link_libraries(localizer_node localization_core ${PCL_LIBRARIES} yaml-cpp)


install(TARGETS localizer_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

endif()

install(
  TARGETS localization_core
  EXPORT localization_core_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(localization_core_targets HAS_LIBRARY_TARGET)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_smoke test/test_smoke.cpp)
  if(TARGET test_smoke)
    ament_target_dependencies(test_smoke interface)
  endif()

  ament_add_gtest(test_localization_state_machine test/test_localization_state_machine.cpp)
  if(TARGET test_localization_state_machine)
    target_link_libraries(test_localization_state_machine localization_core)
  endif()

  ament_add_gtest(test_tf_policy test/test_tf_policy.cpp)
  if(TARGET test_tf_policy)
    target_link_libraries(test_tf_policy localization_core)
  endif()

  ament_add_gtest(test_tile_cache test/test_tile_cache.cpp)
  if(TARGET test_tile_cache)
    target_link_libraries(test_tile_cache localization_core)
  endif()

  ament_add_gtest(test_registration_backend test/test_registration_backend.cpp)
  if(TARGET test_registration_backend)
    target_link_libraries(test_registration_backend localization_core)
  endif()

  ament_add_gtest(test_registration_quality_gate test/test_registration_quality_gate.cpp)
  if(TARGET test_registration_quality_gate)
    target_link_libraries(test_registration_quality_gate localization_core)
  endif()

  ament_add_gtest(test_latest_registration test/test_latest_registration.cpp)
  if(TARGET test_latest_registration)
    target_link_libraries(test_latest_registration localization_core)
  endif()

  ament_add_pytest_test(
    test_tf_owner_launch
    ${CMAKE_CURRENT_SOURCE_DIR}/../test/system/test_tf_owner_launch.py
  )
endif()

ament_package()
