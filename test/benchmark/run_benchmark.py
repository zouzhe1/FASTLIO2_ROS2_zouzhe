#!/usr/bin/env python3
"""Evaluate recorded benchmark metrics against versioned safety/resource gates."""

import argparse
import hashlib
import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path


GATES = {
    "false_trusted_recoveries": ("max", 0),
    "stale_tf_count": ("max", 0),
    "multi_floor_false_matches": ("max", 0),
    "invalid_state_tf_publications": ("max", 0),
    "crashes": ("max", 0),
    "deadlocks": ("max", 0),
    "registration_p95_ms": ("max", 80.0),
    "steady_rss_slope_mb_per_min": ("max", 2.0),
    "recovery_success_rate": ("min", 0.90),
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def software_hash() -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def evaluate(metrics: dict) -> list[dict]:
    decisions = []
    for name, (direction, threshold) in GATES.items():
        value = metrics.get(name)
        passed = value is not None and (
            value <= threshold if direction == "max" else value >= threshold
        )
        decisions.append(
            {"metric": name, "value": value, "gate": direction, "threshold": threshold, "passed": passed}
        )
    return decisions


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--metrics", type=Path)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--bag", type=Path)
    parser.add_argument("--map", dest="map_path", type=Path)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        metrics = {
            "false_trusted_recoveries": 0, "stale_tf_count": 0,
            "multi_floor_false_matches": 0, "invalid_state_tf_publications": 0,
            "crashes": 0, "deadlocks": 0, "registration_p95_ms": 40.0,
            "steady_rss_slope_mb_per_min": 0.1, "recovery_success_rate": 1.0,
        }
        data_source = "synthetic-self-test-not-acceptance"
    elif args.metrics and args.metrics.is_file():
        metrics = json.loads(args.metrics.read_text(encoding="utf-8"))
        data_source = str(args.metrics)
    else:
        metrics = {}
        data_source = "missing"
    decisions = evaluate(metrics)
    provenance = {"software_hash": software_hash(), "metrics_source": data_source}
    for name, path in (("bag_hash", args.bag), ("map_hash", args.map_path), ("config_hash", args.config)):
        provenance[name] = sha256(path) if path and path.is_file() else "not-provided"
    report = {
        "schema_version": 1,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "passed": all(item["passed"] for item in decisions),
        "self_test": args.self_test,
        "provenance": provenance,
        "metrics": metrics,
        "gates": decisions,
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
