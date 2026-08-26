# Performance tuning

Tune with a versioned bag/map/config and change one budget at a time. Recommended order:

1. Reduce source/target point budgets or increase voxel size.
2. Reduce local registration rate while keeping odometry at sensor rate.
3. Adjust the same-level tile-cache count and byte ceilings; keep the normal 3×3
   neighborhood resident when memory permits.
4. Set small_gicp threads to the number that improves measured p95 without starving the
   estimator; two is the conservative default.
5. Change place-recognition top-K only after measuring recall and ambiguity; default is 3.
6. Increase the on-demand recovery deadline only when verified candidates regularly time
   out and front-end CPU/RSS remain stable.

Full/world-map point-cloud publication is disabled by default. Enable it only for short
diagnostics with a subscriber present. Path and loop markers are capped; raising these
limits increases serialization and subscriber memory. Do not loosen quality, ambiguity,
level isolation, correction-jump or LOST/TF safety thresholds merely to improve recall.

Use `test/benchmark/run_benchmark.py` to record hashes and enforce latency, memory slope,
recovery and zero-false-trust gates. Synthetic self-test output validates the harness,
not deployment performance.
