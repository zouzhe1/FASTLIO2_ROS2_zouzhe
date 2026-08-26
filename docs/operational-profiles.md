# Operational profiles

The supported online profiles are mutually exclusive:

- `mapping`: starts FASTLIO2 and PGO. PGO alone owns `map -> odom`.
- `localization`: starts FASTLIO2 and the tiled localizer. The localizer alone owns
  `map -> odom`; PGO is not started.
- `maintenance`: runs offline map validation, tile building or batch optimization.
  It does not publish a live global pose.

Launch mapping with `ros2 launch pgo pgo_launch.py operational_profile:=mapping` and
localization with `ros2 launch localizer localizer_launch.py
operational_profile:=localization`. Unsupported combinations fail during launch or node
startup. FASTLIO2 local odometry remains available after global localization becomes
LOST, but `global_pose_valid` is false and no freshly timestamped stale global TF is sent.

Build a map in two phases. `/pgo/save_maps` creates an immutable keyframe-stream
generation transactionally. Then run `ros2 run map_tools tile_builder_node MAP_ROOT
SOURCE_GENERATION OUTPUT_GENERATION LEVEL_ID`; validate the result with `ros2 run
map_tools map_validator GENERATION_DIR map`. Only a validated tiled generation should
be selected for localization. The `current` pointer identifies the active generation;
older generations remain available for rollback.
