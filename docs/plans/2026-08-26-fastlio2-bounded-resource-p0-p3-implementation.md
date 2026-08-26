# FASTLIO2 Bounded-Resource P0–P3 Optimization Implementation Plan

> Execute task-by-task. A task is complete only after its focused tests and checkpoint verification pass.

**Goal:** Build a platform-neutral, bounded-resource LiDAR–IMU localization and loop-closure system that detects loss, rejects unsafe corrections, and can recover on maps of at least 100,000 m².

**Target environment:** ROS 2 Jazzy on Ubuntu 24.04, C++17, PCL, Eigen, small_gicp, Scan Context, GTSAM/iSAM2, yaml-cpp, ament tests, and rosbag2.

**System boundary:** FASTLIO2 remains the smooth local odometry front end. Mapping, localization, and offline maintenance are mutually exclusive operational profiles. Online localization uses a level-aware 25 m tiled map, bounded caches, asynchronous registration, explicit health states, and one global-TF owner.

---

## Execution Preconditions

Implement on branch `zouzhe_optimize`. Commands assume the repository is located at `$FASTLIO_WS/src/FASTLIO2_ROS2` and ROS 2 Jazzy has been sourced.

Before each task:

```bash
cd "$FASTLIO_WS"
source /opt/ros/jazzy/setup.bash
```

At each P0–P3 checkpoint:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Record the git commit, configuration hash, map ID/generation, bag hash, CPU utilization, peak RSS, registration latency percentiles, localization state transitions, and acceptance result. Do not tune against an undocumented bag.

---

## P0 — Correctness, Safety, and Observability

### Task 1: Establish Interfaces, Tests, and Jazzy CI

**Files:**

- Modify: `interface/CMakeLists.txt`
- Modify: `interface/package.xml`
- Create: `interface/msg/LocalizationStatus.msg`
- Create: `interface/action/Relocalize.action`
- Modify: `localizer/CMakeLists.txt`
- Modify: `localizer/package.xml`
- Modify: `pgo/CMakeLists.txt`
- Modify: `pgo/package.xml`
- Create: `localizer/test/test_smoke.cpp`
- Create: `pgo/test/test_smoke.cpp`
- Create: `.github/workflows/ros2-jazzy.yml`

**Implementation:**

1. Add `ament_cmake_gtest` and package smoke tests before production changes.
2. Generate `LocalizationStatus` with, at minimum:

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
bool global_tf_published
string operational_profile
string tf_owner
string reason
string map_id
string map_hash
uint64 map_generation
builtin_interfaces/Time last_trusted_update
float32 correction_age_sec
float32 correction_translation
float32 correction_rotation_deg
float32 rmse
uint32 inliers
float32 inlier_ratio
float32 overlap
float32 hessian_condition
float32 ambiguity_margin
string level_id
uint64 candidate_id
uint32 active_tiles
uint32 cached_tiles
uint64 cached_bytes
float32 registration_ms
uint32 imu_gap_count
uint32 lidar_gap_count
uint32 timestamp_regression_count
uint32 imu_buffer_size
uint32 lidar_buffer_size
uint64 replaced_work_items
uint32 accepted_loop_count
uint32 rejected_loop_count
uint32 accepted_relocalization_count
uint32 rejected_relocalization_count
```

3. Add a cancellable `Relocalize.action`:

```text
uint8 AUTO=0
uint8 ROUGH_POSE=1
uint8 mode
string map_path
geometry_msgs/PoseWithCovarianceStamped initial_pose
---
bool success
uint8 final_state
string message
geometry_msgs/PoseWithCovarianceStamped pose
---
uint8 state
uint32 candidates_tested
float32 best_score
float32 elapsed_sec
```

4. Add `std_msgs`, `builtin_interfaces`, `geometry_msgs`, and action-generation dependencies explicitly to the interface package.
5. Retain the existing relocalization service only as a compatibility endpoint that reports request acceptance; use the action for completion, cancellation, and feedback.
6. Add an Ubuntu 24.04/Jazzy amd64 build-and-test workflow. Run target-platform performance tests separately when no representative CI runner is available.

**Focused verification:** Interface generation succeeds; message/action types import; smoke tests execute under `colcon test`; the workflow contains no obsolete ROS distribution.

**Commit:** `test: establish localization interfaces and jazzy CI`

### Task 2: Enforce the LiDAR–IMU Sensor Contract and Bound Input Buffers

**Files:**

- Modify: `fastlio2/src/lio_node.cpp`
- Modify: `fastlio2/config/*.yaml`
- Create: `fastlio2/include/fastlio2/sensor_contract.h`
- Create: `fastlio2/src/sensor_contract.cpp`
- Create: `fastlio2/test/test_sensor_contract.cpp`

**Implementation:**

1. Replace the hard-coded accelerometer multiplier with `imu_acc_scale`. Default it to `9.80665` for sources reporting acceleration in g; permit `1.0` for SI-unit sources and document the choice in each sensor profile.
2. Reject or reset safely on non-finite values, non-positive time deltas, timestamp regression, impossible acceleration/angular velocity, empty scans, and excessive sensor gaps.
3. Require stationary initialization variance to be below configurable accelerometer and gyro thresholds before declaring initialization valid.
4. Bound internal deques by both time and count. ROS QoS depth alone does not protect these process-owned buffers.
5. Publish counters and current sizes through `LocalizationStatus`; log the extrinsic/configuration hash once at startup.

Suggested parameters:

```yaml
sensor:
  imu_acc_scale: 9.80665
  max_imu_gap_ms: 20
  max_lidar_gap_ms: 200
  max_imu_buffer_seconds: 2.0
  max_lidar_buffer_frames: 3
  require_stationary_init: true
  max_init_gyro_stddev: 0.02
  max_init_acc_stddev: 0.15
```

Tune gap and variance thresholds from recorded sensor rates; do not silently loosen them after a failure.

**Focused tests:** g-to-SI and SI pass-through; timestamp reset; IMU dropout; saturation; non-finite input; buffer bound; stationary and moving initialization.

**Acceptance:** No unbounded input growth; invalid data cannot produce a fresh trusted global pose; the selected unit profile is observable.

**Commit:** `fix: enforce sensor contract and bounded input buffers`

### Task 3: Add Operational Profiles, Localization State, and One TF Owner

**Files:**

- Modify: `fastlio2/launch/*.launch.py`
- Modify: `localizer/launch/*.launch.py`
- Modify: `pgo/launch/*.launch.py`
- Create: `localizer/include/localizer/localization_state_machine.h`
- Create: `localizer/src/localization_state_machine.cpp`
- Create: `localizer/test/test_localization_state_machine.cpp`
- Create: `localizer/test/test_tf_policy.cpp`
- Create: `test/system/test_tf_owner_launch.py`

**Implementation:**

1. Add a required `operational_profile` launch argument:
   - `mapping`: FASTLIO2 + PGO; PGO is the global-TF owner; localizer is disabled.
   - `localization`: FASTLIO2 + tiled localizer; localizer is the global-TF owner; PGO is disabled.
   - `maintenance`: offline map/HBA tools only; no online global pose claim.
2. Fail launch validation if zero or multiple nodes are configured to own `map -> odom` in an online profile.
3. Implement `UNINITIALIZED -> TRACKING -> DEGRADED -> LOST -> RELOCALIZING -> RECOVERING -> TRACKING` using consecutive-frame counters and hysteresis, not one-scan decisions.
4. Keep local odometry available during `LOST`, but set `global_pose_valid=false`. By default stop publishing fresh `map -> odom`; never republish an old correction with a new timestamp.
5. Resume trusted global TF only after recovery passes the quality gate and motion-consistency confirmation.

**Focused tests:** fake-clock timeout; invalid transitions; hysteresis; LOST TF suppression; stale timestamp prevention; launch tests for each profile and a deliberately conflicting owner configuration.

**Acceptance:** Exactly one global-TF owner in every supported online launch; consumers can distinguish local odometry from a valid global pose.

**Commit:** `feat: add explicit localization and tf ownership policy`

### Task 4: Fix PGO Work Scheduling and Factor Correctness

**Files:**

- Modify: `pgo/src/pgo_node.cpp`
- Modify: `pgo/src/simple_pgo.cpp`
- Create: `pgo/include/pgo/latest_value_slot.h`
- Create: `pgo/test/test_latest_value_slot.cpp`
- Create: `pgo/test/test_factor_noise.cpp`

**Implementation:**

1. Replace unlocked check-then-pop access with a mutex-protected latest-value slot. Overwrite stale pending work and increment `replaced_work_items`.
2. Initialize all timing state explicitly and handle first message/time regression.
3. Decide keyframe admission from pose/time/distance before converting a full `PointCloud2`.
4. Give translational and rotational residuals separate, unit-correct variances; use robust noise for verified loop factors.
5. Avoid recalculating and republishing every optimized pose on every front-end frame; update only after graph changes and publish bounded diagnostics/path history.

**Focused tests:** thread sanitizer where available; concurrent producer/consumer; first timestamp; delayed work overwrite; translation/rotation covariance mapping; no graph update on rejected keyframe.

**Acceptance:** No queue race or uninitialized time use; graph processing cannot lag indefinitely behind live input.

**Commit:** `fix: make pgo scheduling bounded and factor noise correct`

### P0 Checkpoint

Run the full checkpoint commands plus sensor-gap and TF-owner launch tests. P0 passes only when sensor units are explicit, buffers are bounded, LOST cannot masquerade as a valid global pose, and PGO has no known queue race.

---

## P1 — Level-Aware Tiled Map and Bounded Memory

### Task 5: Define the 25 m Level-Aware Tile and Manifest Format

**Files:**

- Create: `map_tools/CMakeLists.txt`
- Create: `map_tools/package.xml`
- Create: `map_tools/include/map_tools/tile_id.h`
- Create: `map_tools/include/map_tools/map_manifest.h`
- Create: `map_tools/src/map_manifest.cpp`
- Create: `map_tools/test/test_tile_id.cpp`
- Create: `map_tools/test/test_map_manifest.cpp`
- Create: `docs/map-format.md`

**Implementation:**

1. Use 25 m horizontal tiles with floor-based indexing for negative coordinates:

```text
tile_x = floor(x / 25.0)
tile_y = floor(y / 25.0)
tile_id = (level_id, tile_x, tile_y)
```

2. Require a stable `level_id`, or deterministic z-band metadata when a semantic floor ID is unavailable. Never merge two similar floors only because their XY indices match.
3. Store manifest version, map ID, generation, frames, tile size, levels/z ranges, tile bounding boxes, point counts, voxel size, checksums, keyframe index reference, and creation/config hashes.
4. Reject incompatible schema, wrong frame, duplicate tile IDs, overlapping level bands beyond policy, checksum mismatch, or missing required files before localization starts.

**Focused tests:** positive/negative boundaries; exact 25 m edge; two levels at the same XY; manifest round trip; corrupt checksum; unsupported version.

**Acceptance:** A map larger than 100,000 m² can be indexed without loading point data, and same-XY multi-floor data remains isolated.

**Commit:** `feat: define level-aware tiled map format`

### Task 6: Implement Streaming, Transactional Map Save and Offline Tile Build

**Files:**

- Modify: `pgo/src/pgo_node.cpp`
- Create: `map_tools/src/tile_builder_node.cpp`
- Create: `map_tools/src/map_validator.cpp`
- Create: `map_tools/test/test_transactional_save.cpp`

**Implementation:**

1. Stop constructing a second full merged point cloud in RAM during save. Stream admitted keyframe patches/poses to a temporary map directory.
2. Build tiles offline or in a bounded worker by transforming one patch at a time, voxelizing per tile, flushing at a byte limit, and releasing source memory.
3. Write checksums and manifest last, fsync where appropriate, validate the temporary generation, then atomically replace the generation pointer/directory.
4. Preserve the last valid generation after disk-full, process interruption, malformed patch, or checksum failure.
5. Keep a merged PCD export as an explicit offline compatibility command, not the normal online save path.

**Focused tests:** forced disk-full/write failure; interruption before manifest; corrupt patch; repeated generation; peak-memory test with synthetic large map.

**Acceptance:** Save peak RSS remains bounded by configured working sets rather than total map size; a failed save cannot destroy the previous map.

**Commit:** `feat: stream and atomically publish map generations`

### Task 7: Add a Bounded Same-Level Tile Cache

**Files:**

- Create: `localizer/include/localizer/tile_cache.h`
- Create: `localizer/src/tile_cache.cpp`
- Create: `localizer/test/test_tile_cache.cpp`
- Modify: `localizer/config/*.yaml`

**Implementation:**

1. Load the current same-level 3 × 3 neighborhood by default; prefetch directionally only when motion and budget permit.
2. Bound cache by both tile count and bytes. Use LRU eviction while pinning tiles referenced by an active registration snapshot.
3. Deduplicate concurrent loads, reject checksum/level mismatch, and return explicit missing/corrupt status.
4. Build reusable small_gicp target structures when a tile neighborhood generation changes, not per scan.

**Focused tests:** LRU order; byte/count limits; pinning; concurrent duplicate request; level switch; missing/corrupt tile; 100,000 m² synthetic traversal.

**Acceptance:** Cache size and peak RSS remain bounded; no registration combines incompatible levels.

**Commit:** `feat: add bounded level-aware tile cache`

### P1 Checkpoint

Validate transactional save/recovery, multi-floor isolation, long synthetic traversal, cache bounds, and map-format rejection. Do not begin P2 until memory consumption is independent of total map size during online localization.

---

## P2 — Efficient Local Tracking and Trust Gate

### Task 8: Add a Preprocessed small_gicp Registration Adapter

**Files:**

- Create: `localizer/include/localizer/registration_backend.h`
- Create: `localizer/src/small_gicp_backend.cpp`
- Create: `localizer/test/test_registration_backend.cpp`
- Modify: `localizer/CMakeLists.txt`
- Modify: `localizer/package.xml`

**Implementation:**

1. Wrap registration behind a narrow backend interface returning transform, convergence, RMSE, inliers, overlap, Hessian/conditioning metric, and elapsed time.
2. Preprocess and reuse the target neighborhood. Bound source points, maximum iterations, correspondence distance, threads, and deadline.
3. Keep the existing PCL ICP path only as a temporary rollback backend through P2 acceptance. Do not maintain two production algorithms after small_gicp passes recorded-bag accuracy and latency gates.
4. Reject non-finite transforms, deadline overruns, excessive correction jumps, and ill-conditioned results.

**Focused tests:** known transform; low overlap; planar degeneracy; empty input; deadline; target preprocessing reuse; parity benchmark against the temporary backend.

**Acceptance:** small_gicp meets accuracy gates while lowering or matching p95 latency; backend cannot allocate/work without configured bounds.

**Commit:** `feat: add bounded preprocessed small-gicp backend`

### Task 9: Add the Quality Gate and Asynchronous Local Tracking

**Files:**

- Create: `localizer/include/localizer/registration_quality_gate.h`
- Create: `localizer/src/registration_quality_gate.cpp`
- Modify: `localizer/src/localizer_node.cpp`
- Create: `localizer/test/test_registration_quality_gate.cpp`
- Create: `localizer/test/test_latest_registration.cpp`

**Implementation:**

1. Evaluate RMSE, inlier count/ratio, overlap, conditioning, correction jump, deadline, tile completeness, and predicted motion. No single scalar is sufficient.
2. Use state-dependent thresholds: tracking may use a tight predicted-pose search; recovery requires stronger evidence and an ambiguity margin.
3. Run local registration asynchronously with one in-flight job and one latest pending request. Tag inputs/results with generation IDs and discard obsolete results.
4. Apply accepted corrections through a bounded `map -> odom` update policy. Large but valid recovery corrections require the recovery confirmation path instead of entering tracking directly.
5. Publish reason codes for every rejection and all metrics needed to reproduce the decision.

**Focused tests:** each rejection reason; threshold hysteresis; outdated result; worker overrun; abrupt false match; DEGRADED/LOST transition; recovery jump routing.

**Acceptance:** Front-end callbacks never wait for registration; stale/ambiguous results cannot update global TF; CPU use stays bounded during overload.

**Commit:** `feat: gate and schedule local registration safely`

### P2 Checkpoint

Run representative straight, turning, stop/start, sparse geometry, repetitive corridor, and temporary occlusion bags. Compare baseline and optimized accuracy, p50/p95/p99 latency, CPU, RSS, and state transitions. Remove the PCL rollback backend after the agreed parity window; document any temporary retention with an expiry condition.

---

## P3 — Shared Place Recognition, Relocalization, and Loop Verification

### Task 10: Add Shared Online and Prebuilt-Map Place Recognition (P3)

**Files:**

- Create: `place_recognition/CMakeLists.txt`
- Create: `place_recognition/package.xml`
- Create: `place_recognition/include/place_recognition/place_index.h`
- Create: `place_recognition/src/place_index.cpp`
- Create: `place_recognition/test/test_place_index.cpp`
- Modify: `map_tools/src/tile_builder_node.cpp`
- Modify: `localizer/src/localizer_node.cpp`
- Modify: `pgo/src/pgo_node.cpp`

**Implementation:**

1. Build one shared library for descriptor generation, index serialization, candidate filtering, and scoring. The localizer and PGO must not drift into separate Scan Context implementations.
2. Store descriptor version/config hash, keyframe pose, yaw hint, `level_id`, associated tiles, session ID, timestamp, and checksum.
3. For prebuilt maps, generate the index offline. For online mapping, add only admitted keyframes and bound retained raw clouds.
4. Filter candidates by descriptor score, temporal/session rules, level compatibility, and optional rough-pose radius. Return top-K with scores; do not claim a pose from descriptor similarity alone.
5. Set default `K=3`, configurable and benchmarked. Cache/load the compact index independently from tile point data.

**Focused tests:** serialization compatibility; descriptor/config mismatch; same-place retrieval; repeated structure ambiguity; same-XY different-level rejection; online/offline descriptor parity.

**Acceptance:** Candidate lookup latency and memory are bounded for the acceptance map; both loop closure and relocalization consume the same tested implementation.

**Commit:** `feat: share bounded place recognition index`

### Task 11: Add On-Demand Global Recovery and Robust Loop Verification (P3)

**Files:**

- Create: `localizer/include/localizer/global_recovery.h`
- Create: `localizer/src/global_recovery.cpp`
- Create: `localizer/test/test_global_recovery.cpp`
- Create: `pgo/include/pgo/loop_verifier.h`
- Create: `pgo/src/loop_verifier.cpp`
- Create: `pgo/test/test_loop_verifier.cpp`
- Modify: `localizer/src/localizer_node.cpp`
- Modify: `pgo/src/simple_pgo.cpp`

**Core recovery pipeline:**

```text
Scan Context top-3
  -> candidate keyframe pose and yaw
  -> candidate-associated same-level tiles
  -> coarse bounded GICP
  -> fine bounded GICP
  -> multi-metric quality gate
  -> best-versus-second ambiguity margin
  -> three motion-consistent frames
  -> RECOVERING then TRACKING
```

**Implementation:**

1. Trigger automatic search when a ready map has no trusted initial pose and after confirmed `LOST`; also accept manual requests through `Relocalize.action`. Keep local odometry running while global validity is false.
2. In `ROUGH_POSE` mode, bypass descriptor search and begin bounded registration from the supplied 6DoF pose. Both modes use the same tile loader, quality gate, and temporal confirmation.
3. Process candidates sequentially under total time, memory, point, and iteration budgets. Support cancellation between stages.
4. Never concatenate the entire map. Load only candidate-associated same-level tiles and release them after scoring.
5. Require absolute quality thresholds and a configurable best/second score margin. If candidates remain ambiguous, stay LOST and report the ambiguity.
6. After a winning candidate, require three consecutive scans whose accepted poses agree with local odometry motion before restoring global validity/TF.
7. Reuse the same candidate and geometric verification pipeline for PGO loop proposals. Add a robust graph factor only after verification; blacklist repeatedly rejected pairs for a bounded interval.
8. Re-optimize/publish only when a verified factor changes the graph; guard large corrections with diagnostics and configured limits.

**Conditional P3b, not part of the default core:** Evaluate a heavy correspondence/global-registration backend only if versioned acceptance bags demonstrate that the candidate-pose coarse/fine GICP pipeline misses required recoveries. Admission requires a measured recall improvement large enough to justify its p95 latency, peak RSS, dependency, and maintenance cost. It remains on-demand and budgeted, never per-frame.

**Focused tests:** known kidnapping; false top-1/valid top-2; best/second ambiguity; wrong level; repetitive scene; cancellation; total deadline; three-frame consistency; false loop rejection; robust-factor outlier; bounded blacklist.

**Acceptance:** Recovery cannot create a trusted pose from descriptor score alone; false positive rate meets the acceptance gate; failed recovery does not block front-end odometry or grow memory.

**Commit:** `feat: add bounded relocalization and verified loops`

### Task 12: Bound ROS Transport, Histories, and Published Products

**Files:**

- Modify: `fastlio2/src/lio_node.cpp`
- Modify: `localizer/src/localizer_node.cpp`
- Modify: `pgo/src/pgo_node.cpp`
- Modify: `fastlio2/config/*.yaml`
- Modify: `localizer/config/*.yaml`
- Modify: `pgo/config/*.yaml`
- Create: `test/system/test_transport_bounds.py`

**Implementation:**

1. Use sensor-data QoS or small best-effort depths for high-rate clouds/IMU; use reliable depth 1–5 for control/status as appropriate. Remove oversized publisher histories.
2. Bound path length, debug clouds, keyframe clouds, diagnostic history, and visualization rate. Disable world/full-map cloud publication by default.
3. Publish static/prebuilt metadata once with appropriate durability; do not retransmit large map products per frame.
4. Expose current internal queue/buffer sizes and drop/overwrite counts so transport and process backpressure are distinguishable.

**Focused tests:** slow subscriber; disconnected subscriber; visualization enabled/disabled; history upper bounds; sustained sensor rate; no increase in RSS after warm-up beyond tolerance.

**Acceptance:** Slow or absent subscribers cannot cause unbounded process memory or stall the estimator.

**Commit:** `perf: bound transport and visualization histories`

### Task 13: Add Versioned Bag Benchmarks and Fault Injection

**Files:**

- Create: `test/benchmark/README.md`
- Create: `test/benchmark/manifest.yaml`
- Create: `test/benchmark/run_benchmark.py`
- Create: `test/fault_injection/*.py`
- Create: `docs/acceptance-criteria.md`

**Required scenarios:**

- At least one map covering 100,000 m² or more, with a 30-minute continuous run.
- Open area, repetitive corridor, turns, stops, dynamic occlusion, sparse geometry, and revisit loops.
- Kidnapping/restart from multiple map regions, including near tile and level boundaries.
- Same-XY multi-floor scenes designed to tempt a false match.
- IMU delay/drop/saturation, LiDAR gaps, timestamp reset/regression, and initialization while moving.
- Corrupt/missing tile, wrong map/index pair, config-hash mismatch, disk full, and interrupted map save.

**Metrics and gates:**

- Accuracy: ATE/RPE or surveyed checkpoints, loop correction error, recovery position/yaw error.
- Safety: false trusted recovery count, stale-TF count, multi-floor false matches, invalid-state TF publication.
- Recovery: recall, success rate, time-to-recover, candidates tested, ambiguity rejection.
- Performance: total/per-node CPU, peak and steady RSS, callback/registration p50/p95/p99, cache hit rate, drops/overwrites.
- Stability: crashes, deadlocks, state flapping, queue growth, and 30-minute memory slope.

Every result must name the bag/map/config/software hashes. Establish numerical gates from the current working baseline and application requirements before declaring a phase complete; never replace safety gates with relative improvement alone.

**Acceptance:** One command produces a machine-readable report and a non-zero exit code for gate failure.

**Commit:** `test: add localization performance and fault gates`

### Task 14: Complete Configuration, Migration, and Final Verification

**Files:**

- Modify: `README.md`
- Modify: package launch/config files
- Create: `docs/operational-profiles.md`
- Create: `docs/relocalization.md`
- Create: `docs/performance-tuning.md`
- Modify: `.github/workflows/ros2-jazzy.yml`

**Implementation:**

1. Ship conservative defaults for mapping/localization/maintenance and validate mutually inconsistent parameters at startup.
2. Document map building/validation, map ID and generation handling, initial pose, action feedback/cancellation, LOST behavior, TF ownership, and rollback.
3. Document performance tuning in priority order: point budget/voxel size, local registration rate, tile cache, thread count, candidate count, then recovery deadline. Preserve safety thresholds unless benchmark evidence supports a change.
4. Remove obsolete launch options, duplicate production registration code, stale documentation, and hidden hardware-specific assumptions.
5. Run full unit/system tests, all acceptance bags, fault injection, clean-install launch, and map migration/rollback.

**Acceptance:** A new operator can select a profile and map without source edits; unsupported combinations fail clearly; CI and representative target results pass.

**Commit:** `docs: finalize bounded localization operations`

### P3 Checkpoint

P3 passes only when automatic/manual relocalization, verified loops, long-run resource bounds, fault injection, and public operational documentation all satisfy the recorded gates.

---

## Separate Offline Optimization Track

HBA and other batch refinements remain offline maintenance tools. Their numerical hardening, checkpoint/restart support, and large-map optimization are useful but must not block P0–P3 online safety. They do not publish live global TF and are benchmarked against map-quality improvement and offline resource/time budgets.

---

## Final Definition of Done

- Exactly one node owns global TF in each online profile.
- LiDAR/IMU units, time behavior, initialization, extrinsics, gaps, and buffers are explicit and observable.
- LOST never appears as a fresh valid global pose; recovery requires geometric evidence, ambiguity rejection, and temporal consistency.
- Online memory and work queues are bounded independently of total map size.
- Map save is streaming and transactional; the previous valid generation survives failure.
- 25 m XY tiles are isolated by level/z metadata and validated before use.
- small_gicp reuses preprocessed targets and meets recorded latency/accuracy gates; duplicate fallback code is retired.
- Loop closure and relocalization share place recognition and geometric verification.
- Heavy global registration is introduced only after measured evidence, and only on demand.
- Versioned bags, maps, configs, CI, and fault tests make every stability/performance claim reproducible.
