# Localization benchmark

`run_benchmark.py` turns metrics collected from a versioned rosbag2 run into a JSON
gate report and returns non-zero when any required metric is missing or fails. Example:

```bash
python3 test/benchmark/run_benchmark.py \
  --metrics results/metrics.json --bag bags/campus.db3 \
  --map maps/campus/generation-4/manifest.yaml \
  --config localizer/config/localizer.yaml --report results/report.json
```

`--self-test` checks the reporting mechanism only. It is marked synthetic in the
report and is not evidence that localization meets field acceptance. The checked-in
manifest intentionally has `acceptance_ready: false` until a 100,000 m², 30-minute
bag and surveyed truth are supplied without redistribution/licensing problems.
