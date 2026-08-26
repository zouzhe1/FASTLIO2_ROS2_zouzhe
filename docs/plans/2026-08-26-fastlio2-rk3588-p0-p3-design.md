# FASTLIO2 RK3588 P0–P3 Optimization Design

## 1. Purpose

This design hardens the existing FASTLIO2 ROS2 repository for online use on RK3588 with LiDAR and IMU only. It targets mixed environments, including indoor warehouses, long corridors, underground parking, tunnels, and outdoor areas, with a single map covering at least 100,000 m².

The system cannot mathematically guarantee drift-free localization under every LiDAR/IMU observability failure. The engineering target is therefore:

- keep local odometry smooth and real-time;
- bound global drift while map matching is healthy;
- detect degradation instead of silently publishing stale global corrections;
- recover automatically after loss;
- reject false loop closures before they can corrupt the pose graph;
- keep online CPU and memory bounded as the full map grows.

## 2. Scope

### Included

- P0 correctness and safety fixes in `pgo`, `localizer`, and `hba`;
- P1 25 m map tiling, bounded local-map cache, latest-only work queues, and message-path reduction;
- P2 small_gicp-based local registration, registration quality gates, asynchronous workers, and localization health states;
- P3 Scan Context loop/relocalization candidate retrieval, on-demand global registration, robust loop factors, and incremental pose updates;
- RK3588 performance instrumentation and bag-based acceptance tests.

### Excluded

- GNSS/RTK, UWB, wheel odometry, cameras, and fixed landmarks;
- feeding global correction jumps back into the FASTLIO2 filter state;
- online HBA execution;
- learned point-cloud descriptors or an always-running NPU/GPU pipeline;
- replacement of the FASTLIO2 front end.

## 3. Selected Architecture

The selected approach is a balanced CPU-first pipeline:

```text
LiDAR + IMU
    |
    v
FASTLIO2 front end ----> smooth odom and body cloud
    |                              |
    |                              v
    |                      keyframe admission
    |                              |
    |                 +------------+-------------+
    |                 |                          |
    |                 v                          v
    |       25 m tiled local-map cache     Scan Context database
    |                 |                          |
    |                 v                          v
    |       low-rate local GICP          top-k loop/recovery candidates
    |                 |                          |
    |                 +------------+-------------+
    |                              |
    |                     registration quality gate
    |                              |
    +------------------------------+----> map->odom correction
                                   |
                                   v
                         robust incremental iSAM2
```

FASTLIO2 remains independent. Controllers consume the smooth `odom` frame, while planning and map-level consumers use the globally corrected `map` frame. Loop closure may move `map->odom`; it must not introduce discontinuities into the local odometry stream.

## 4. Map Tiling and Cache

### 4.1 Tile coordinates

The fixed tile size is 25 m:

```text
tile_x = floor(world_x / 25.0)
tile_y = floor(world_y / 25.0)
```

Floor semantics are required so negative coordinates map correctly. Tile filenames use signed integer coordinates, for example `tile_-2_7.pcd`.

### 4.2 On-disk representation

The production map is a directory:

```text
map_directory/
  manifest.yaml
  tiles/
    tile_0_0.pcd
    tile_0_1.pcd
    ...
```

`manifest.yaml` records format version, frame, tile size, source resolution, map bounds, and available tile coordinates. A build utility converts a monolithic PCD into this representation. Monolithic PCD input remains available as a compatibility path but is not the production path for maps at or above 100,000 m².

### 4.3 Active window

Normal tracking loads a 3×3 tile window centered on the predicted pose, covering approximately 75 m × 75 m. A configurable prefetch margin loads the next neighbor before crossing a tile boundary. Hysteresis prevents repeated loading when the pose oscillates around a boundary.

The cache stores downsampled points, normals/covariances, and registration search structures. It has a hard tile-count and byte limit. Eviction uses least-recently-used order and never evicts a tile referenced by the current registration job.

## 5. Scheduling and Data Ownership

- ROS callbacks perform timestamp checks, pose extraction, and keyframe admission only.
- Point-cloud conversion occurs only after a message is admitted as a keyframe or selected as the latest localization scan.
- PGO and localization workers each own a capacity-one latest-value slot. A new value replaces an unprocessed old value.
- FASTLIO2 never waits for localization, loop detection, map I/O, or HBA.
- Local-map construction, GICP, Scan Context query, and global recovery run outside ROS subscription and service callbacks.
- Map save and HBA remain explicit offline/maintenance operations.

The implementation must define one mutex per owned state object. A field may not be written under one mutex and read under another.

## 6. Localization Health State Machine

```text
UNINITIALIZED -> RELOCALIZING -> RECOVERING -> TRACKING
                                      ^           |
                                      |           v
                                     LOST <- DEGRADED
```

### State behavior

- `UNINITIALIZED`: no global pose is valid.
- `TRACKING`: local GICP passes all quality gates; `map->odom` may be updated.
- `DEGRADED`: recent matches are weak; retain the last correction, increase reported uncertainty, and expand the candidate/local-map search without blocking FASTLIO2.
- `LOST`: global pose is explicitly invalid. Continue publishing local odometry, but do not claim that the old global correction is current.
- `RELOCALIZING`: Scan Context and on-demand global registration are searching for recovery candidates.
- `RECOVERING`: a candidate passed one geometric check, but must pass three consecutive keyframe checks before becoming trusted.

Initial transition policy:

- two consecutive weak/failed local matches: `TRACKING -> DEGRADED`;
- five consecutive failures or five seconds without a trusted update: `DEGRADED -> LOST`;
- one global candidate plus three consistent local matches: `LOST -> RECOVERING -> TRACKING`.

All counts and durations are YAML parameters.

## 7. Registration and Quality Gates

### 7.1 Normal tracking

Normal tracking uses a low-rate local GICP/VGICP registration against the cached 3×3 local map. Initial settings for RK3588 are:

- update rate: 1 Hz;
- worker threads: 2;
- source point budget: 5,000–15,000;
- target point budget: 30,000–80,000;
- rough resolution: 0.4–0.5 m;
- fine resolution: 0.15–0.25 m;
- rough maximum correspondence distance: 1–2 m;
- fine maximum correspondence distance: 0.3–0.8 m.

These are tuning ranges, not fixed acceptance values.

### 7.2 Acceptance gate

A transformation is trusted only when all enabled checks pass:

- registration converged;
- minimum inlier count;
- minimum inlier ratio;
- minimum estimated overlap;
- maximum RMSE;
- bounded translation and rotation correction for the current state;
- finite, symmetric, sufficiently conditioned 6×6 Hessian;
- consistency with recent FASTLIO2/IMU motion;
- temporal consistency during recovery.

Every rejection records a machine-readable reason.

### 7.3 Global recovery and loop closure

Scan Context is computed for admitted keyframes and retrieves the top three candidates while excluding temporally adjacent frames. Scan Context proposes candidates only; it never creates a graph factor by itself.

In `LOST` or initial localization, a robust global registration stage runs on demand, followed by GICP refinement and the same quality gate. This heavy path must not run continuously during `TRACKING`.

## 8. Pose Graph Policy

- Odometry factors use separate rotation and translation noise, not a shared scalar.
- Loop registration produces a regularized symmetric information matrix or calibrated diagonal covariance.
- Loop factors are wrapped in a configurable robust model such as Cauchy, Huber, or DCS.
- A loop is added only after descriptor retrieval, geometry verification, and temporal/motion consistency checks.
- iSAM2 updates the newest pose incrementally on ordinary keyframes.
- Recalculating and copying every historical pose occurs only after an accepted loop, explicit map save, or visualization request.
- Full-resolution keyframe clouds are not retained indefinitely in RAM. The online graph stores pose, descriptor, quality summary, and a bounded/downsampled registration cloud.

## 9. HBA Policy

HBA is removed from the online launch path. Before it remains available as an offline tool, P0 must:

- clear previous input on a new refine request;
- validate external text/PCD input without relying on `assert`;
- clamp hierarchy levels to a valid range;
- propagate all configured BLAM parameters;
- add a gauge-fixing prior;
- construct valid symmetric positive-definite factor information;
- report running, success, and failure states accurately.

## 10. RK3588 Performance Budget

- FASTLIO2 processing P99 is below one LiDAR frame period.
- Local GICP runs at 1 Hz by default and has P95 latency at or below 250 ms on the target dataset.
- Local GICP uses at most two Cortex-A76 worker threads by default.
- Backend pending work never exceeds one item per worker.
- Tile I/O and search-structure construction never execute on the FASTLIO2 callback path.
- Online process RSS stays below 2 GB for the 100,000 m² acceptance map.
- Thirty-minute tracking shows no monotonic unbounded memory growth.
- Global recovery targets three seconds on representative revisits; hard scenes may remain explicitly `LOST` rather than publish an unsafe pose.
- HBA is excluded from the online budget.

CPU affinity is deployment policy rather than hard-coded core numbering. The deployment should reserve high-performance cores for FASTLIO2 and at most two high-performance cores for registration, leaving lightweight ROS I/O and diagnostics on efficiency cores where practical.

## 11. Observability and Diagnostics

Publish a localization status message containing:

- state enum;
- global pose validity;
- last trusted update time;
- failure/rejection reason;
- inliers, inlier ratio, overlap, RMSE, and Hessian condition estimate;
- active and cached tile counts;
- local/global registration latency;
- dropped/replaced pending work count;
- accepted/rejected loop counts.

The status is part of the acceptance interface. A consumer must be able to distinguish smooth local odometry from trusted global localization.

## 12. Verification Strategy

### Unit tests

- tile indexing for positive, negative, and boundary coordinates;
- 3×3 active window, prefetch hysteresis, and LRU eviction;
- capacity-one latest-value behavior;
- state-machine transitions and timeout behavior;
- quality-gate accept/reject reasons;
- covariance/information regularization;
- loop candidate temporal exclusion and top-k ordering;
- HBA input reset and invalid-file handling.

### Bag and system tests

- warehouse with repeated aisles;
- long corridor/tunnel with geometric degeneracy;
- underground parking with similar levels;
- open outdoor area;
- correct loop, false loop candidate, and no-loop sequence;
- initial pose offsets and yaw errors;
- kidnapped/restart recovery;
- dynamic occlusion and sparse scans;
- at least 30 minutes continuous operation;
- at least 100,000 m² tiled map.

Record ATE/RPE when ground truth exists, loop precision/recall, start/end closure error, relocalization success rate, false acceptance rate, recovery time, P50/P95/P99 latency, CPU, RSS, temperature, and pending-work replacements.

## 13. Rollout and Rollback

Each phase is separately switchable in YAML:

- P0 ships first and must not change intended nominal trajectories except for fixing invalid behavior.
- P1 can fall back to monolithic-map compatibility mode during validation.
- P2 can select PCL ICP or small_gicp behind one adapter.
- P3 candidate retrieval and global recovery can be disabled independently.

The current public topics and services remain available during migration. New status fields and tiled-map support are additive until the final compatibility cleanup.
