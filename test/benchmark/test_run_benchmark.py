import json
import subprocess
import sys
from pathlib import Path


def test_self_test_writes_machine_readable_passing_report(tmp_path):
    script = Path(__file__).with_name("run_benchmark.py")
    report = tmp_path / "report.json"
    completed = subprocess.run(
        [sys.executable, str(script), "--self-test", "--report", str(report)],
        check=False,
    )
    assert completed.returncode == 0
    data = json.loads(report.read_text(encoding="utf-8"))
    assert data["passed"] is True
    assert data["provenance"]["software_hash"]
    assert data["metrics"]["false_trusted_recoveries"] == 0


def test_gate_failure_returns_nonzero(tmp_path):
    script = Path(__file__).with_name("run_benchmark.py")
    metrics = tmp_path / "metrics.json"
    report = tmp_path / "report.json"
    metrics.write_text(json.dumps({"false_trusted_recoveries": 1}), encoding="utf-8")
    completed = subprocess.run(
        [sys.executable, str(script), "--metrics", str(metrics), "--report", str(report)],
        check=False,
    )
    assert completed.returncode != 0
    assert json.loads(report.read_text(encoding="utf-8"))["passed"] is False
