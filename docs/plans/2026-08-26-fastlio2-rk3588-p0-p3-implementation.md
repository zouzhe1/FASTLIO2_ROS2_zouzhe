# FASTLIO2 RK3588 P0–P3 Optimization Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Build a bounded-resource FASTLIO2 localization and loop-closure backend for RK3588 that detects loss, rejects unsafe corrections, and recovers globally on maps of at least 100,000 m².

**Architecture:** Keep FASTLIO2 as an independent smooth odometry front end. Feed admitted keyframes into asynchronous local registration and loop workers backed by a 25 m tiled map, Scan Context candidate retrieval, small_gicp refinement, explicit localization health states, and robust incremental GTSAM factors. Keep heavy global recovery on demand and HBA offline.

**Tech Stack:** ROS2 Humble, C++17, PCL, Eigen, small_gicp, Scan Context, GTSAM/iSAM2, yaml-cpp, ament_cmake_gtest, rosbag2.

---

## Execution Preconditions

Run implementation in a dedicated worktree on a `bobo/` branch. Commands below assume this repository is inside a ROS2 workspace at `$FASTLIO_WS/src/FASTLIO2_ROS2` on Ubuntu 22.04 with ROS2 Humble sourced.

Before every task:

```bash
cd "$FASTLIO_WS"
source /opt/ros/humble/setup.bash
```

After every task, run the package-specific tests shown in that task. At P0, P1, P2, and P3 checkpoints, also run:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Expected checkpoint result: build succeeds and all tests pass with zero failed tests.

### Task 1: Establish Test and Diagnostics Foundations

**Files:**
- Modify: `interface/CMakeLists.txt`
- Modify: `interface/package.xml`
- Create: `interface/msg/LocalizationStatus.msg`
- Modify: `localizer/CMakeLists.txt`
- Modify: `localizer/package.xml`
- Modify: `pgo/CMakeLists.txt`
- Modify: `pgo/package.xml`
- Create: `localizer/test/test_smoke.cpp`
- Create: `pgo/test/test_smoke.cpp`

**Step 1: Add failing interface-generation expectation**

Create `LocalizationStatus.msg` with the final message shape:

```text
uint8 UNINITIALIZED=0
uint8 TRACKING=1
uint8 DEGRADED=2
uint8 LOST=3
uint8 RELOCALIZING=4
uint8 RECOVERING=5

std_msgs/Header header
uint8 state
bool global_pose_valid
string reason
float32 rmse
uint32 inliers
float32 inlier_ratio
float32 overlap
float32 hessian_condition
float32 registration_ms
uint32 active_tiles
uint32 cached_tiles
uint64 replaced_work_items
```

Add the message to `rosidl_generate_interfaces()` and add `std_msgs` as a dependency.

**Step 2: Add minimal failing package smoke tests**

Each package test should include its primary public header and contain one test named `PackageSmoke.HeadersCompile`. Add `ament_cmake_gtest` under `BUILD_TESTING`.

**Step 3: Run the tests and confirm the expected initial failure**

```bash
colcon build --packages-select interface localizer pgo --symlink-install
```

Expected before CMake/package wiring is complete: message generation or test target configuration fails.

**Step 4: Complete CMake and package dependencies**

Use `rosidl_default_generators` for the message and export `rosidl_default_runtime`. Ensure the test targets link the package libraries introduced by later tasks rather than node executables.

**Step 5: Verify**

```bash
colcon build --packages-select interface localizer pgo --symlink-install
colcon test --packages-select interface localizer pgo --event-handlers console_direct+
colcon test-result --verbose
```

Expected: build and smoke tests pass.

**Step 6: Commit**

```bash
git add interface localizer/CMakeLists.txt localizer/package.xml localizer/test pgo/CMakeLists.txt pgo/package.xml pgo/test
git commit -m "test: establish localization backend test harness"
```

### Task 2: Fix PGO Timestamp and Latest-Only Input Handling (P0)

**Files:**
- Create: `pgo/src/pgos/latest_value_slot.h`
- Create: `pgo/test/test_latest_value_slot.cpp`
- Modify: `pgo/src/pgo_node.cpp`

**Step 1: Write failing latest-value tests**

Cover empty `take()`, one insert, replacement, replacement counter, and concurrent producer/consumer safety. The core assertions are:

```cpp
LatestValueSlot<int> slot;
EXPECT_FALSE(slot.take().has_value());
slot.put(1);
slot.put(2);
EXPECT_EQ(slot.replacedCount(), 1U);
EXPECT_EQ(slot.take().value(), 2);
EXPECT_FALSE(slot.take().has_value());
```

**Step 2: Run the focused test and verify failure**

```bash
colcon test --packages-select pgo --ctest-args -R test_latest_value_slot --output-on-failure
```

Expected: compile failure because `LatestValueSlot` does not exist.

**Step 3: Implement the minimal slot**

Implement a mutex-protected `std::optional<T>` with `put(T)`, `take()`, `hasValue()`, and `replacedCount()`. `put()` increments the counter only when replacing pending work.

**Step 4: Replace the PGO queue**

- initialize timestamp state explicitly with `std::optional<double>`;
- reject out-of-order messages only after a previous timestamp exists;
- perform keyframe pose admission before `pcl::fromROSMsg`;
- put only admitted keyframes into `LatestValueSlot<CloudWithPose>`;
- make `timerCB()` atomically take the newest item;
- remove lock-free `size()` and `front()` access.

**Step 5: Verify unit tests and build**

```bash
colcon build --packages-select pgo --symlink-install
colcon test --packages-select pgo --event-handlers console_direct+
colcon test-result --verbose
```

Expected: all PGO tests pass; ThreadSanitizer job, when enabled in CI, reports no queue race.

**Step 6: Commit**

```bash
git add pgo/src pgo/test
git commit -m "fix: process only the latest valid PGO keyframe"
```

### Task 3: Add Explicit Localization State and Correct Lock Ownership (P0)

**Files:**
- Create: `localizer/src/localizers/localization_state_machine.h`
- Create: `localizer/src/localizers/localization_state_machine.cpp`
- Create: `localizer/test/test_localization_state_machine.cpp`
- Modify: `localizer/src/localizer_node.cpp`
- Modify: `localizer/config/localizer.yaml`

**Step 1: Write failing transition tests**

Test the configured initial policy:

```cpp
LocalizationStateMachine sm({2, 5, 5.0, 3});
EXPECT_EQ(sm.state(), LocalizationState::UNINITIALIZED);
sm.startRelocalization();
EXPECT_EQ(sm.state(), LocalizationState::RELOCALIZING);
sm.onGlobalCandidateAccepted();
EXPECT_EQ(sm.state(), LocalizationState::RECOVERING);
sm.onTrustedLocalMatch();
sm.onTrustedLocalMatch();
sm.onTrustedLocalMatch();
EXPECT_EQ(sm.state(), LocalizationState::TRACKING);
```

Also test weak-match transition, failure timeout, reset, and that one good frame cannot recover from `LOST`.

**Step 2: Verify the tests fail**

```bash
colcon test --packages-select localizer --ctest-args -R test_localization_state_machine --output-on-failure
```

Expected: compile failure because the state machine is absent.

**Step 3: Implement the state machine as a pure C++ class**

Do not include ROS types. Inject timestamps as `double` seconds so tests use deterministic time.

**Step 4: Refactor node state ownership**

- use one mutex for service/relocalization requests;
- use one mutex for the latest synchronized scan/odom snapshot;
- copy snapshots under lock, then release the lock before point-cloud processing;
- prevent map reload from racing with registration;
- publish `LocalizationStatus` every state change and at 1 Hz;
- make `Relocalize` mean “request accepted/map accepted,” not “pose solved”;
- keep `relocalize_check` for compatibility but derive it from the state machine.

**Step 5: Add YAML parameters**

```yaml
state:
  degraded_after_failures: 2
  lost_after_failures: 5
  lost_after_seconds: 5.0
  recovery_confirm_frames: 3
```

**Step 6: Verify**

```bash
colcon build --packages-select interface localizer --symlink-install
colcon test --packages-select localizer --event-handlers console_direct+
colcon test-result --verbose
```

Expected: tests pass and node compiles with a single-threaded and multi-threaded executor build configuration.

**Step 7: Commit**

```bash
git add interface localizer
git commit -m "fix: make localization health explicit and thread safe"
```

### Task 4: Correct PGO Noise Construction and HBA Lifecycle (P0)

**Files:**
- Create: `pgo/src/pgos/noise_model_factory.h`
- Create: `pgo/src/pgos/noise_model_factory.cpp`
- Create: `pgo/test/test_noise_model_factory.cpp`
- Modify: `pgo/src/pgos/simple_pgo.cpp`
- Modify: `pgo/src/pgos/simple_pgo.h`
- Modify: `hba/src/hba_node.cpp`
- Modify: `hba/src/hba/hba.cpp`
- Modify: `hba/src/hba/hba.h`
- Create: `hba/test/test_hba_input.cpp`
- Modify: `hba/CMakeLists.txt`
- Modify: `hba/package.xml`

**Step 1: Write failing information-matrix tests**

Test that a non-symmetric, indefinite, or near-singular input produces a finite symmetric positive-definite matrix after regularization. Test separate rotational/translational fallback variances and robust-kernel selection.

```cpp
Eigen::Matrix<double, 6, 6> bad = Eigen::Matrix<double, 6, 6>::Zero();
bad(0, 1) = 10.0;
auto result = regularizeInformation(bad, config);
EXPECT_TRUE(result.isApprox(result.transpose(), 1e-9));
EXPECT_GT(Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>>(result)
              .eigenvalues().minCoeff(), 0.0);
```

**Step 2: Run and confirm failure**

```bash
colcon test --packages-select pgo hba --event-handlers console_direct+
```

Expected: new tests fail before the factory and reset APIs exist.

**Step 3: Implement calibrated robust loop noise**

- stop using ICP fitness as all six variances;
- accept a registration Hessian/quality summary when available;
- symmetrize and eigenvalue-clamp information;
- provide configured diagonal fallback sigmas;
- wrap loop noise with Cauchy by default;
- keep odometry rotation and translation sigmas separate.

**Step 4: Fix HBA lifecycle and graph gauge**

- add `HBA::clear()` and call it before loading a new request;
- replace external-input `assert` with checked parsing and error responses;
- clamp hierarchy levels to at least one when input is sufficient;
- propagate `plane_thresh` into `BLAMConfig`;
- add a strong but finite prior on the first pose;
- construct factor information from valid symmetric blocks, never directly from an off-diagonal Hessian block;
- reject invalid factors with a diagnostic rather than letting GTSAM fail later.

**Step 5: Verify P0 checkpoint**

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Expected: zero failed tests.

**Step 6: Commit**

```bash
git add pgo hba
git commit -m "fix: harden pose graph and offline HBA inputs"
```

### Task 5: Implement 25 m Tile Indexing and Manifest I/O (P1)

**Files:**
- Create: `localizer/src/localizers/tile_index.h`
- Create: `localizer/src/localizers/tile_index.cpp`
- Create: `localizer/src/localizers/tile_manifest.h`
- Create: `localizer/src/localizers/tile_manifest.cpp`
- Create: `localizer/test/test_tile_index.cpp`
- Create: `localizer/test/test_tile_manifest.cpp`

**Step 1: Write failing coordinate tests**

Include exact boundaries and negative values:

```cpp
EXPECT_EQ(tileFor(0.0, 0.0, 25.0), (TileId{0, 0}));
EXPECT_EQ(tileFor(24.999, 24.999, 25.0), (TileId{0, 0}));
EXPECT_EQ(tileFor(25.0, 0.0, 25.0), (TileId{1, 0}));
EXPECT_EQ(tileFor(-0.001, 0.0, 25.0), (TileId{-1, 0}));
EXPECT_EQ(tileFor(-25.0, -25.0, 25.0), (TileId{-1, -1}));
```

Test 3×3 neighbors, deterministic filenames, manifest round trip, version rejection, and missing-tile behavior.

**Step 2: Verify failure**

```bash
colcon test --packages-select localizer --ctest-args -R "test_tile_(index|manifest)" --output-on-failure
```

Expected: compile failure before the tile classes exist.

**Step 3: Implement pure indexing and YAML manifest code**

Use `std::floor`, signed 32-bit tile coordinates, deterministic ordering, and explicit manifest schema version `1`.

**Step 4: Verify**

```bash
colcon build --packages-select localizer --symlink-install
colcon test --packages-select localizer --event-handlers console_direct+
```

Expected: all tile tests pass.

**Step 5: Commit**

```bash
git add localizer/src/localizers localizer/test
git commit -m "feat: add versioned 25 meter map tile indexing"
```

### Task 6: Add Offline Map Tile Builder (P1)

**Files:**
- Create: `localizer/src/map_tile_builder.cpp`
- Create: `localizer/test/test_map_tile_builder.cpp`
- Modify: `localizer/CMakeLists.txt`
- Create: `docs/map-tiling.md`

**Step 1: Write a failing fixture test**

Generate a small synthetic cloud crossing positive and negative tile boundaries. Run the builder library function into a temporary directory, then assert point ownership, manifest bounds, and that no point appears in two tiles.

**Step 2: Run and verify failure**

```bash
colcon test --packages-select localizer --ctest-args -R test_map_tile_builder --output-on-failure
```

Expected: failure because the builder is absent.

**Step 3: Implement the builder**

Provide a ROS-independent library function and a thin CLI:

```text
map_tile_builder --input map.pcd --output map_tiles --tile-size 25 \
  --rough-resolution 0.5 --fine-resolution 0.2
```

Write into a temporary sibling directory and atomically rename only after all tiles and `manifest.yaml` are valid. Preserve the source PCD.

**Step 4: Add documentation and verification command**

Document directory layout, disk-space estimate, re-running behavior, and a `--verify-only` mode.

**Step 5: Verify**

```bash
colcon build --packages-select localizer --symlink-install
colcon test --packages-select localizer --event-handlers console_direct+
ros2 run localizer map_tile_builder --help
```

Expected: tests pass and CLI usage is printed.

**Step 6: Commit**

```bash
git add localizer docs/map-tiling.md
git commit -m "feat: build versioned tiled localization maps"
```

### Task 7: Build the Bounded Tiled-Map Cache (P1)

**Files:**
- Create: `localizer/src/localizers/tiled_map_cache.h`
- Create: `localizer/src/localizers/tiled_map_cache.cpp`
- Create: `localizer/test/test_tiled_map_cache.cpp`
- Modify: `localizer/config/localizer.yaml`

**Step 1: Write failing cache tests**

Test 3×3 active-window selection, boundary prefetch, hysteresis, LRU order, byte/tile limits, pinned-tile protection, missing tiles, and cache reuse when the center tile is unchanged.

**Step 2: Verify failure**

```bash
colcon test --packages-select localizer --ctest-args -R test_tiled_map_cache --output-on-failure
```

Expected: compile failure before the cache exists.

**Step 3: Implement the cache with injected I/O**

Inject a tile loader interface so unit tests do not touch PCD files. Return immutable shared tile handles containing fine/rough point sets and preprocessed registration structures. Do not hold the cache mutex during disk reads or preprocessing.

**Step 4: Add initial configuration**

```yaml
map:
  tile_size: 25.0
  active_radius_tiles: 1
  prefetch_margin: 5.0
  hysteresis: 2.0
  max_cached_tiles: 25
  max_cache_bytes: 1073741824
```

**Step 5: Verify P1 checkpoint**

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Expected: zero failed tests and cache tests prove bounded tile count/bytes.

**Step 6: Commit**

```bash
git add localizer
git commit -m "feat: cache bounded local map tiles"
```

### Task 8: Introduce a Registration Adapter and small_gicp (P2)

**Files:**
- Create: `localizer/src/localizers/registration_types.h`
- Create: `localizer/src/localizers/registration_backend.h`
- Create: `localizer/src/localizers/pcl_icp_backend.h`
- Create: `localizer/src/localizers/pcl_icp_backend.cpp`
- Create: `localizer/src/localizers/small_gicp_backend.h`
- Create: `localizer/src/localizers/small_gicp_backend.cpp`
- Create: `localizer/test/test_registration_quality.cpp`
- Create: `localizer/test/test_registration_backends.cpp`
- Modify: `localizer/CMakeLists.txt`
- Modify: `localizer/package.xml`
- Modify: `localizer/config/localizer.yaml`

**Step 1: Pin and build small_gicp in the ROS2 workspace**

Record the selected upstream release/commit in `docs/dependencies.md`. Use `find_package(small_gicp REQUIRED)`; do not fetch changing `master` during the build.

**Step 2: Write backend contract tests**

The shared result must contain:

```cpp
struct RegistrationResult {
  bool converged;
  Eigen::Isometry3d transform;
  double rmse;
  std::size_t inliers;
  double inlier_ratio;
  double overlap;
  Eigen::Matrix<double, 6, 6> hessian;
  std::size_t iterations;
  double elapsed_ms;
};
```

Test identity alignment, known transform, insufficient points, no overlap, and reuse of a preprocessed target.

**Step 3: Verify failure**

```bash
colcon test --packages-select localizer --ctest-args -R test_registration --output-on-failure
```

Expected: compile failure before the adapters exist.

**Step 4: Implement compatibility and small_gicp backends**

- preserve PCL ICP behind the interface for rollback;
- preprocess each immutable target tile/window once;
- use small_gicp GICP or VGICP with two threads by default;
- cap source and target point counts;
- expose complete quality data;
- avoid rebuilding unchanged target search structures.

**Step 5: Add configuration**

```yaml
registration:
  backend: small_gicp
  type: VGICP
  threads: 2
  update_hz: 1.0
  rough_resolution: 0.5
  fine_resolution: 0.2
  rough_max_correspondence: 1.5
  fine_max_correspondence: 0.5
  max_source_points: 15000
  max_target_points: 80000
```

**Step 6: Verify**

```bash
colcon build --packages-select localizer --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select localizer --event-handlers console_direct+
```

Expected: both backends pass common correctness tests.

**Step 7: Commit**

```bash
git add localizer docs/dependencies.md
git commit -m "feat: add reusable small_gicp registration backend"
```

### Task 9: Implement Quality Gates and Asynchronous Local Tracking (P2)

**Files:**
- Create: `localizer/src/localizers/registration_quality_gate.h`
- Create: `localizer/src/localizers/registration_quality_gate.cpp`
- Create: `localizer/src/localizers/localization_worker.h`
- Create: `localizer/src/localizers/localization_worker.cpp`
- Create: `localizer/test/test_registration_quality_gate.cpp`
- Create: `localizer/test/test_localization_worker.cpp`
- Modify: `localizer/src/localizer_node.cpp`
- Modify: `localizer/config/localizer.yaml`

**Step 1: Write table-driven quality tests**

Cover every rejection reason independently: no convergence, too few inliers, low inlier ratio, low overlap, high RMSE, excessive correction, non-finite Hessian, poor condition, and motion inconsistency. Test that all checks must pass.

**Step 2: Write worker scheduling tests**

Inject fake registration and tile-cache implementations. Prove that callbacks are non-blocking, newest input replaces old input, map reload cancels/invalidates old work, and state transitions follow Task 3.

**Step 3: Verify failure**

```bash
colcon test --packages-select localizer --ctest-args -R "test_(registration_quality_gate|localization_worker)" --output-on-failure
```

Expected: compile failure before the gate and worker exist.

**Step 4: Implement the gate and worker**

The worker owns the registration backend and map snapshot. It posts results to the node through a result slot; the node applies a correction only if the result references the current map generation and passes the gate.

**Step 5: Add configurable initial thresholds**

```yaml
quality:
  min_inliers: 800
  min_inlier_ratio: 0.25
  min_overlap: 0.20
  max_rmse: 0.35
  max_tracking_translation: 1.5
  max_tracking_rotation_deg: 10.0
  max_hessian_condition: 1000000.0
```

These are starting values and must be tuned from bag data.

**Step 6: Verify P2 checkpoint**

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Expected: zero failed tests; fake slow registration does not delay subscription callback tests.

**Step 7: Commit**

```bash
git add localizer
git commit -m "feat: gate and schedule local registration safely"
```

### Task 10: Add Scan Context Candidate Retrieval (P3)

**Files:**
- Create: `pgo/src/descriptors/scan_context_adapter.h`
- Create: `pgo/src/descriptors/scan_context_adapter.cpp`
- Create: `pgo/src/pgos/loop_candidate.h`
- Create: `pgo/test/test_scan_context_adapter.cpp`
- Create: `pgo/test/test_loop_candidate_filter.cpp`
- Modify: `pgo/src/pgos/simple_pgo.h`
- Modify: `pgo/src/pgos/simple_pgo.cpp`
- Modify: `pgo/config/pgo.yaml`
- Modify: `pgo/CMakeLists.txt`
- Create: `docs/dependencies-scan-context.md`

**Step 1: Pin Scan Context and preserve its license**

Vendor or reference a fixed upstream commit, record its URL/commit/license, and wrap it so upstream types do not leak through the PGO public API.

**Step 2: Write candidate tests**

Test descriptor insertion, top-three ordering, temporal exclusion, empty database, reverse-yaw candidate handling, and that a descriptor match alone cannot create a loop factor.

**Step 3: Verify failure**

```bash
colcon test --packages-select pgo --ctest-args -R "test_(scan_context|loop_candidate)" --output-on-failure
```

Expected: compile failure before the adapter exists.

**Step 4: Integrate candidate generation**

- compute descriptors only for admitted keyframes;
- query at a configurable maximum rate;
- retrieve top three candidates;
- exclude recent frames by both elapsed time and keyframe distance;
- enqueue candidates for geometry verification;
- retain position-radius search as an optional secondary candidate source, not the sole detector.

**Step 5: Add configuration**

```yaml
scan_context:
  enabled: true
  top_k: 3
  exclude_recent_seconds: 60.0
  exclude_recent_keyframes: 30
  query_hz: 1.0
```

**Step 6: Verify and commit**

```bash
colcon build --packages-select pgo --symlink-install
colcon test --packages-select pgo --event-handlers console_direct+
git add pgo docs/dependencies-scan-context.md
git commit -m "feat: retrieve global loop candidates with Scan Context"
```

### Task 11: Add On-Demand Global Recovery and Robust Loop Verification (P3)

**Files:**
- Create: `localizer/src/localizers/global_registration_backend.h`
- Create: `localizer/src/localizers/global_recovery_worker.h`
- Create: `localizer/src/localizers/global_recovery_worker.cpp`
- Create: `localizer/test/test_global_recovery_worker.cpp`
- Create: `pgo/src/pgos/loop_verifier.h`
- Create: `pgo/src/pgos/loop_verifier.cpp`
- Create: `pgo/test/test_loop_verifier.cpp`
- Modify: `localizer/src/localizer_node.cpp`
- Modify: `pgo/src/pgo_node.cpp`
- Modify: `pgo/src/pgos/simple_pgo.cpp`
- Modify: `pgo/config/pgo.yaml`

**Step 1: Write fake-backend recovery tests**

Prove that global work starts only in `UNINITIALIZED`, `RELOCALIZING`, or `LOST`; it is cancelled on map generation change; one candidate moves to `RECOVERING`; and three consistent local matches are required before `TRACKING`.

**Step 2: Write loop-verifier tests**

Test top-k candidates where the best descriptor is geometrically wrong, a later candidate is valid, all candidates fail, transforms violate motion bounds, Hessians are degenerate, and temporal consistency rejects a one-frame false positive.

**Step 3: Verify failure**

```bash
colcon test --packages-select localizer pgo --ctest-args -R "test_(global_recovery|loop_verifier)" --output-on-failure
```

Expected: compile failure before workers/verifier exist.

**Step 4: Implement the on-demand path**

- define a global-registration interface so KISS-Matcher/Quatro/TEASER++ can be selected without changing node logic;
- implement the first selected backend as a pinned optional dependency;
- refine every accepted global result with the Task 8 GICP backend;
- apply the Task 9 quality gate;
- enforce a CPU concurrency limit so global and local registration cannot both saturate RK3588;
- continue publishing local odometry while global pose validity is false.

**Step 5: Integrate robust loop factors**

Pass the verified transform and regularized quality information to `NoiseModelFactory`. Add the factor only after all checks pass. Record accepted and rejected candidate metrics and rejection reasons.

**Step 6: Optimize ordinary iSAM2 updates**

On a non-loop keyframe, calculate only the latest pose estimate. Refresh all stored global poses only after an accepted loop, explicit save, or requested visualization snapshot.

**Step 7: Verify P3 checkpoint and commit**

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --event-handlers console_direct+
colcon test-result --verbose
git add localizer pgo
git commit -m "feat: recover globally and verify loops robustly"
```

### Task 12: Bound ROS2 Transport, Visualization, and Stored History

**Files:**
- Modify: `fastlio2/src/lio_node.cpp`
- Modify: `pgo/src/pgo_node.cpp`
- Modify: `pgo/src/pgos/simple_pgo.h`
- Modify: `localizer/src/localizer_node.cpp`
- Modify: `fastlio2/config/lio.yaml`
- Modify: `pgo/config/pgo.yaml`
- Create: `pgo/test/test_keyframe_retention.cpp`

**Step 1: Write retention tests**

Test bounded in-memory registration clouds, descriptor/pose retention, disk-backed or evicted clouds, and that saving a map reports missing data instead of dereferencing an evicted cloud.

**Step 2: Implement bounded transport and visualization**

- use sensor-data QoS with depth 1–5 for clouds;
- avoid queue depth 10,000;
- limit or disable full-path publication in production;
- publish static/full maps once with transient-local QoS rather than serializing them every registration cycle;
- disable unused world-cloud/path visualization by configuration;
- publish downsampled keyframe clouds to the backend;
- retain full clouds only according to a configured memory/disk policy.

**Step 3: Verify**

```bash
colcon build --packages-select fastlio2 pgo localizer --symlink-install
colcon test --packages-select pgo localizer --event-handlers console_direct+
```

Expected: tests pass and queue/history limits are visible in diagnostics.

**Step 4: Commit**

```bash
git add fastlio2 pgo localizer
git commit -m "perf: bound cloud transport and keyframe history"
```

### Task 13: Add RK3588 Benchmark and Bag-Regression Tooling

**Files:**
- Create: `tools/benchmark/run_bag_benchmark.py`
- Create: `tools/benchmark/collect_metrics.py`
- Create: `tools/benchmark/compare_results.py`
- Create: `tools/benchmark/scenarios.yaml`
- Create: `tools/benchmark/README.md`
- Create: `test/system/README.md`
- Create: `docs/rk3588-performance-acceptance.md`

**Step 1: Write failing parser/comparison tests**

Create synthetic metrics JSON for baseline and candidate runs. Test P50/P95/P99 computation, RSS growth detection, loop precision/recall, recovery-time calculation, threshold failure, and missing-metric handling.

**Step 2: Implement deterministic benchmark output**

Each run writes:

```text
results/<scenario>/<git-sha>/
  metadata.json
  latency.json
  resource.json
  localization.json
  loops.json
  summary.md
```

Record git SHA, YAML hashes, bag hash, board model, kernel, ROS middleware, thread count, temperature, and throttling state.

**Step 3: Define acceptance scenarios**

Include warehouse, corridor/tunnel, parking, outdoor, false-loop, kidnapped recovery, dynamic occlusion, 30-minute soak, and 100,000 m² map scenarios. A scenario may be marked pending only with an owner and missing dataset description.

**Step 4: Add acceptance gates**

- FASTLIO2 P99 below one sensor period;
- local GICP P95 at or below 250 ms;
- online RSS below 2 GB;
- no monotonic RSS growth in the soak test;
- pending queues never exceed one;
- no accepted false loop in the adversarial set;
- global recovery target at or below three seconds on representative revisits;
- accuracy does not regress beyond the configured baseline tolerance.

**Step 5: Run on development machine, then RK3588**

```bash
python3 tools/benchmark/run_bag_benchmark.py --scenario warehouse --output results
python3 tools/benchmark/compare_results.py --baseline results/baseline --candidate results/candidate
```

Expected: the comparison exits non-zero for any failed acceptance gate and writes `summary.md`.

**Step 6: Commit**

```bash
git add tools test/system docs/rk3588-performance-acceptance.md
git commit -m "test: add RK3588 localization performance gates"
```

### Task 14: Complete Documentation, Migration, and Final Verification

**Files:**
- Modify: `README.md`
- Create: `docs/localization-status.md`
- Create: `docs/p0-p3-migration.md`
- Create: `docs/rk3588-deployment.md`
- Create: `docs/loop-and-relocalization-tuning.md`
- Modify: `localizer/launch/localizer_launch.py`
- Modify: `pgo/launch/pgo_launch.py`
- Modify: `hba/launch/hba_launch.py`

**Step 1: Document operational truth**

Clearly distinguish:

- smooth but locally drifting `odom`;
- globally corrected `map`;
- valid and invalid global localization states;
- online PGO/localizer versus offline HBA;
- monolithic-map compatibility versus tiled production maps.

**Step 2: Document migration**

Provide exact commands to tile an existing map, launch with PCL compatibility, launch with small_gicp, enable Scan Context, inspect status, and roll back each phase by YAML.

**Step 3: Verify launch and configuration parsing**

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch fastlio2 lio_launch.py --show-args
ros2 launch localizer localizer_launch.py --show-args
ros2 launch pgo pgo_launch.py --show-args
```

Expected: every launch file resolves without missing parameters or packages.

**Step 4: Run full verification**

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
git diff --check origin/main...HEAD
git status --short --branch
```

Expected: zero failed tests, no whitespace errors, and only intended changes are present.

**Step 5: Run the RK3588 acceptance matrix**

Execute every available scenario in `tools/benchmark/scenarios.yaml`. Do not mark P0–P3 complete if a required scenario lacks results; document missing datasets as blocked acceptance work.

**Step 6: Commit**

```bash
git add README.md docs localizer/launch pgo/launch hba/launch
git commit -m "docs: complete RK3588 P0-P3 deployment guide"
```

## Final Delivery Criteria

P0–P3 is complete only when:

- all unit and integration tests pass;
- every required benchmark gate passes on RK3588;
- the 100,000 m² tiled map runs with bounded memory;
- false-loop adversarial tests produce zero accepted false loops;
- localization state accurately reports degradation and loss;
- recovery tests demonstrate `LOST -> RECOVERING -> TRACKING` without blocking FASTLIO2;
- online launch excludes HBA;
- rollback switches and migration steps are documented and tested.
