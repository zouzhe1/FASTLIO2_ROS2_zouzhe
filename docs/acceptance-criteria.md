# Localization acceptance criteria

Field acceptance requires a versioned ROS 2 bag, map generation, configuration and
software commit. The primary dataset must cover at least 100,000 m² and 30 minutes,
including open, repetitive, sparse, dynamic, multi-level and kidnapped/restart cases.

Safety gates are absolute: zero trusted false recoveries, stale TF, cross-level false
matches, or global TF publications from an invalid state. The run must have zero crash
or deadlock. Default performance gates are registration p95 ≤ 80 ms and post-warm-up
RSS slope ≤ 2 MB/min. Recovery success must be at least 90%; deployment-specific
position/yaw and ATE/RPE limits must be set from surveyed requirements before release.

Every report records bag/map/config/software hashes. Relative improvement is useful
for tuning but cannot waive a safety gate. Missing metrics fail closed. The repository
currently contains the harness and synthetic tests, not representative field bags;
therefore field acceptance remains unproven until those artifacts are run.
