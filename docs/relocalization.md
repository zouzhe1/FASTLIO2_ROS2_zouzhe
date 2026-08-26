# Relocalization

The compatibility `relocalize` service only confirms request admission. New integrations
should use the cancellable `interface/action/Relocalize` action at
`/localizer/relocalize_action`.

`AUTO` loads the compact place index, searches top-3 candidates, loads only their
associated same-level tiles, and geometrically verifies each candidate with bounded
GICP. A result is rejected when absolute quality or best-versus-second ambiguity fails.
`ROUGH_POSE` skips descriptor lookup and loads the 3×3 same-level tile neighborhood
around the supplied 6DoF pose. Both modes require three motion-consistent accepted scans
before global validity and TF return.

```bash
ros2 action send_goal --feedback /localizer/relocalize_action \
  interface/action/Relocalize \
  "{mode: 0, map_path: '/maps/site', initial_pose: {header: {frame_id: map}, pose: {pose: {orientation: {w: 1.0}}}}}"
```

Cancellation is checked between candidates and while waiting for temporal confirmation.
On cancellation, timeout, corrupt/missing tiles, wrong level, ambiguity or failed
geometry, the system stays globally invalid and local odometry continues. Monitor
`localization_status` for state, reason, candidate, map generation, quality metrics,
latency and accepted/rejected counters.
