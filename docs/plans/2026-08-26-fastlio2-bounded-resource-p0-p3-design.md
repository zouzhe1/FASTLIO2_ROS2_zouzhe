# FASTLIO2 Bounded-Resource P0–P3 Optimization Design

## 1. Purpose

This design hardens the existing FASTLIO2 ROS 2 Jazzy repository for bounded-resource LiDAR+IMU mapping and localization. It targets warehouses, corridors, underground and multi-level parking, tunnels, and outdoor areas, with a single map covering at least 100,000 m².

No LiDAR+IMU system can guarantee zero drift under every observability failure. The engineering contract is instead:

- keep local `odom` smooth and real-time;
- correct global drift when the fixed map is observable;
- detect sensor, registration, and place-recognition degradation;
- never present an old or rejected global pose as newly trusted;
- reject ambiguous loops and relocalization candidates;
- recover automatically when the environment provides enough geometry;
- bound CPU, memory, queues, map I/O, and stored history.

## 2. Engineering Review Decisions

The implementation is intentionally split into operational profiles. PGO and fixed-map localization do not run as competing global correction owners.

### Required profiles

```text
MAPPING
LiDAR + IMU -> FASTLIO2 -> PGO/loop closure -> saved patches and poses
                                                   |
                                                   v
                                  offline tiled map + place index

LOCALIZATION
LiDAR + IMU -> FASTLIO2 -> tiled-map localizer -> map->odom
                              |          |
                              |          +-> LOST recovery
                              +-> health/status

MAINTENANCE
saved map -> optional offline HBA / verification / export
```

Rules:

- `MAPPING`: PGO is the only global TF owner; localizer is disabled.
- `LOCALIZATION`: localizer is the only global TF owner; PGO is disabled.
- `MAINTENANCE`: no online localization claim is made.
- Launch validation fails when more than one node is configured to publish the same global TF.

This separation removes duplicate registration, duplicate graph work, and conflicting `map->odom` updates.

### Included

- P0 sensor-unit, timing, queue, synchronization, lock, TF ownership, and status correctness;
- P0 PGO queue and factor-noise safety fixes;
- P1 25 m horizontal tiles with height/level separation, bounded cache, and streaming map output;
- P2 asynchronous small_gicp tracking with reusable target preprocessing and quality gates;
- P3 shared Scan Context candidate retrieval for online loops and prebuilt-map recovery;
- P3 candidate-pose coarse GICP, fine GICP, ambiguity rejection, and multi-frame confirmation;
- platform-neutral performance and bag-regression acceptance tests.

### NOT in scope for the core P0–P3 delivery

- simultaneous PGO and fixed-map localizer global correction;
- online HBA;
- making offline HBA hardening block online localization delivery;
- always-running KISS-Matcher, Quatro, TEASER++, learned descriptors, GPU, or NPU pipelines;
- automatic mutation of the production map during localization;
- GNSS/RTK, cameras, UWB, wheel odometry, or fixed landmarks;
- feeding global correction jumps into the FASTLIO2 filter state;
- replacement of the FASTLIO2 front end.

A heavy global-registration backend is a conditional P3b item. It is added only if the accepted datasets prove that Scan Context candidate poses plus coarse-to-fine GICP do not meet recovery targets.

## 3. What Already Exists

- FASTLIO2 already publishes smooth LiDAR+IMU odometry and body-frame point clouds; it remains the local estimator.
- PGO already admits key poses, runs radius-prior ICP, uses iSAM2, saves patches/poses, and publishes a correction; the plan hardens rather than replaces it.
- localizer already accepts a rough 6DoF pose, performs two-stage ICP, and computes `map->odom`; the plan preserves this compatibility path.
- the current map-save output already contains viewpoint-bearing patches and poses when requested; the offline builder consumes these instead of inventing a parallel mapping format.
- HBA already consumes saved patches and poses; it stays an offline optional tool.

## 4. Sensor and Front-End Contract

Backend optimization cannot repair incorrect LiDAR/IMU input. P0 therefore defines an explicit sensor contract:

- configure the IMU acceleration scale instead of hard-coding it; for Livox raw acceleration in g, the default conversion is `9.80665` m/s² per g;
- record expected IMU and LiDAR rates, maximum gaps, timestamp regression limits, and initialization variance thresholds;
- reject negative propagation intervals and empty scans before accessing the last point;
- bound IMU history by time span and LiDAR history by frame count;
- detect IMU saturation, large time gaps, out-of-order data, and non-stationary initialization;
- report the active extrinsic calibration and its hash in diagnostics;
- prevent a sensor fault from silently appearing as a valid global-registration failure.

Sensor callbacks remain short. FASTLIO2 processing retains priority over mapping, localization, visualization, and map I/O.

## 5. Map Tiling, Height Separation, and Storage

### 5.1 Horizontal tile coordinates

The default horizontal tile size is 25 m:

```text
tile_x = floor(world_x / 25.0)
tile_y = floor(world_y / 25.0)
```

The size is configurable but one manifest uses one value. Floor semantics are required for negative coordinates.

### 5.2 Height and level identity

An XY tile is not sufficient for multi-level parking or stacked warehouse floors. Each tile record also has:

- a stable `level_id`, when the map builder has explicit level metadata; or
- a `z_min/z_max` band generated from configured height bands.

Candidate retrieval and local-map loading must pass a compatible level/height gate. Automatic recovery never merges geometrically similar floors into one target cloud.

Example filename: `tile_-2_7_level_1.pcd`.

### 5.3 On-disk representation

```text
map_directory/
  manifest.yaml
  levels/
    level_0/
      tiles/
        tile_0_0_level_0.pcd
  keyframes/
    keyframe_000001.pcd
  place_index/
    descriptors.bin
    search_index.bin
    keyframes.yaml
```

The manifest records schema version, coordinate frame, tile size, height policy, levels/bands, resolutions, bounds, tile inventory, source-data hash, keyframe inventory, place-index hash, and capability flags.

The preferred input is saved keyframe patches and poses. A monolithic PCD may be split for manual rough-pose localization, but it is marked `automatic_global_relocalization: false` unless suitable viewpoint-bearing source data exists.

### 5.4 Streaming and transactional output

- Mapping saves patches and poses incrementally or streams them without constructing a second full map in RAM.
- A merged `map.pcd` is an optional offline export, not a required online artifact.
- Builders write to a temporary sibling directory, verify all hashes, then atomically rename.
- Disk-full, permission, malformed pose, missing patch, and interrupted-build failures preserve the previous valid map.

### 5.5 Active window

Normal localization uses a 3×3 horizontal tile window on the active level, approximately 75 m × 75 m. Prefetch and hysteresis avoid boundary stalls and repeated loads.

The cache stores downsampled points, covariances/normals, and registration search structures. It has hard tile-count and byte limits. Jobs pin immutable tile handles; I/O and preprocessing occur outside cache locks.

## 6. Localization Health State Machine

```text
UNINITIALIZED -> RELOCALIZING -> RECOVERING -> TRACKING
                                       ^           |
                                       |           v
                                      LOST <- DEGRADED
```

State behavior:

- `UNINITIALIZED`: no global pose exists and no global TF is published.
- `TRACKING`: registration passes every enabled quality and freshness gate.
- `DEGRADED`: retain the last correction temporarily, increase uncertainty, and publish its age.
- `LOST`: `global_pose_valid=false`; stop publishing fresh `map->odom` by default while local `odom` continues.
- `RELOCALIZING`: automatic or rough-pose candidate evaluation is active.
- `RECOVERING`: one candidate passed geometry but is not trusted until temporal confirmation finishes.

Default transitions:

- two consecutive weak/failed local matches: `TRACKING -> DEGRADED`;
- five failures or five seconds without a trusted update: `DEGRADED -> LOST`;
- accepted candidate plus three motion-consistent keyframes: `LOST -> RECOVERING -> TRACKING`.

The compatibility service reports only that a request was accepted. A ROS 2 action reports progress, cancellation, timeout, and the final trusted result.

## 7. Registration and Quality Gates

### 7.1 Normal tracking

Normal tracking uses low-rate small_gicp GICP/VGICP against the preprocessed active window:

- update rate: 1 Hz default;
- worker threads: at most two by default;
- source budget: 5,000–15,000 points;
- target budget: 30,000–80,000 points;
- rough resolution: 0.4–0.5 m;
- fine resolution: 0.15–0.25 m.

The PCL ICP backend remains a time-limited rollback option through P2 acceptance. Remove the duplicate production backend after small_gicp passes correctness and bag-regression gates.

### 7.2 Acceptance gate

A correction is trusted only when all enabled checks pass:

- registration converged;
- minimum inliers, inlier ratio, and overlap;
- maximum RMSE;
- bounded translation and rotation change for the current state;
- finite symmetric Hessian with acceptable conditioning;
- compatible level/height;
- consistency with recent FASTLIO2/IMU motion;
- current map generation and sensor snapshot;
- fresh result within its deadline.

Each rejection produces a stable machine-readable reason.

### 7.3 Global recovery and loop closure

One ROS-independent `place_recognition` library serves PGO online history and the localizer's versioned prebuilt-map index.

The core recovery path is:

```text
Scan Context top-3
  -> map keyframe pose + yaw estimate
  -> load associated level and neighboring tiles
  -> coarse GICP
  -> fine GICP
  -> quality gate
  -> best/second-best ambiguity margin
  -> three motion-consistent keyframes
  -> TRACKING
```

Scan Context never creates a graph factor or trusted localization by itself. Repetitive corridors and similar floors must fail closed when candidates remain ambiguous.

The rough-pose path bypasses Scan Context and starts from the supplied 6DoF pose. Both paths use the same tile loader, registration backend, gate, and recovery confirmation.

P3b heavy global registration is compiled or enabled only after benchmark evidence shows a recovery gap. It is cancellable, rate-limited, and never runs during healthy `TRACKING`.

## 8. Pose Graph Policy

- PGO runs only in `MAPPING` mode.
- Odometry factors use separately calibrated rotational and translational noise.
- Verified loop information is regularized and wrapped in a robust model.
- Descriptor retrieval, geometry, ambiguity, level, motion, and temporal gates precede a loop factor.
- Ordinary iSAM2 updates calculate the newest estimate; all historical poses refresh only after an accepted loop, explicit save, or requested snapshot.
- Full-resolution keyframe clouds do not remain indefinitely in RAM.

## 9. HBA Policy

HBA is excluded from all online profiles and performance budgets. Its input validation, gauge fixing, factor regularization, and lifecycle cleanup remain valuable, but belong to a separate offline-maintenance workstream and do not block P0–P3 localization delivery.

## 10. Bounded-Resource Performance Budget

- FASTLIO2 P99 processing stays below one LiDAR frame period.
- Internal LiDAR and IMU buffers remain within configured frame/time bounds.
- Local GICP defaults to 1 Hz, two workers, and P95 latency at or below 250 ms on the acceptance dataset.
- Local and global work share a concurrency budget; recovery cannot starve FASTLIO2.
- Each worker has at most one pending latest-value item.
- Tile I/O, map save, place-index work, and target preprocessing never run on sensor callbacks.
- Online RSS remains below the configured deployment profile limit; the initial reference gate is 2 GB for the 100,000 m² map.
- Streaming map save does not create an additional full-map-sized memory spike.
- Thirty-minute tracking has no monotonic unbounded memory or queue growth.
- Recovery targets three seconds on representative revisits; ambiguous scenes remain explicitly `LOST`.

CPU affinity is deployment policy, not hard-coded core numbering.

## 11. Observability and TF Contract

`LocalizationStatus` contains:

- state and `global_pose_valid`;
- whether global TF is currently published;
- active operational profile and TF owner;
- map ID/hash and generation;
- last trusted update timestamp and correction age;
- last correction translation/rotation;
- inliers, overlap, RMSE, Hessian condition, and ambiguity margin;
- selected level/height band and candidate ID;
- active/cached tile counts and bytes;
- local/recovery latency and pending-work replacements;
- IMU/LiDAR gaps, timestamp regressions, buffer sizes, and sensor-contract failures;
- accepted/rejected loop and relocalization counts with reason codes.

Local `odom` remains available in every health state. Navigation and map-level consumers must gate global use on `global_pose_valid`; they must not infer trust from the presence of a stale transform.

## 12. Verification Strategy

### Unit and component tests

- IMU scale, stationary initialization, gaps, timestamp regression, empty scans, and bounded buffers;
- exactly one global TF owner per launch profile;
- LOST stops fresh global TF while local odometry continues;
- tile indexing for negative/boundary coordinates and level/height isolation;
- manifest/index hashes, map generation, interrupted builds, and atomic replacement;
- 3×3 cache, prefetch, hysteresis, byte limits, pinning, and eviction;
- capacity-one scheduling, cancellation, stale-result rejection, and deadlines;
- registration quality and ambiguity rejection;
- Scan Context top-k ordering, same-floor gating, and false-candidate geometry rejection;
- robust graph information and newest-only iSAM2 updates.

### Bag and system tests

- warehouse repeated aisles;
- long corridor/tunnel degeneracy;
- multi-level parking with similar floors;
- open outdoor area;
- correct loop, false loop, and no-loop sequences;
- rough-pose offsets and yaw errors;
- unknown-pose startup and kidnapped recovery;
- dynamic occlusion and map changes;
- IMU delay/dropout, timestamp reset, LiDAR gaps, and invalid extrinsics;
- map/index mismatch, missing tile, corrupt patch, permission denial, and disk-full simulation;
- 30-minute soak and 100,000 m² map;
- mapping and localization launch-profile exclusivity.

Record ATE/RPE where ground truth exists, loop/relocalization precision and recall, false acceptance, recovery time, P50/P95/P99 latency, CPU, RSS, queue depth, sensor faults, and correction age.

No phase is accepted without versioned bags, configuration hashes, map hashes, and stored baseline results.

## 13. Rollout

1. P0 establishes sensor, queue, lock, TF, state, and diagnostics correctness using existing ICP.
2. P1 adds level-aware tiled maps, bounded cache, and streaming transactional map output.
3. P2 introduces asynchronous small_gicp and removes the PCL production fallback after acceptance.
4. P3a adds shared place recognition, loop verification, and candidate-pose GICP recovery.
5. P3b heavy global registration is added only when accepted data demonstrates a remaining recovery gap.
6. Offline HBA improvements proceed independently.

Every phase is controlled by YAML/launch configuration and has an explicit rollback path. Public compatibility services remain during migration, but their semantics are corrected and documented.
