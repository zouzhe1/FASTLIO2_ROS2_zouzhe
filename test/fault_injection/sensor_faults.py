#!/usr/bin/env python3
"""Deterministically mutate JSONL sensor fixtures without touching the source."""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path); parser.add_argument("output", type=Path)
    parser.add_argument("--drop-every", type=int, default=0)
    parser.add_argument("--delay-ms", type=float, default=0.0)
    parser.add_argument("--regress-at", type=int, default=-1)
    parser.add_argument("--saturate-at", type=int, default=-1)
    args = parser.parse_args()
    if args.output.exists(): raise SystemExit("output already exists")
    with args.input.open(encoding="utf-8") as source, args.output.open("x", encoding="utf-8") as output:
        for index, line in enumerate(source):
            if args.drop_every and (index + 1) % args.drop_every == 0: continue
            item = json.loads(line); item["timestamp"] += args.delay_ms / 1000.0
            if index == args.regress_at: item["timestamp"] -= 10.0
            if index == args.saturate_at: item["values"] = [1e9] * len(item.get("values", []))
            output.write(json.dumps(item) + "\n")


if __name__ == "__main__": main()
