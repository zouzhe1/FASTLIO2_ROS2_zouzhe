#!/usr/bin/env python3
"""Create a separate corrupted map fixture; never mutates the source generation."""
import argparse
import shutil
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path); parser.add_argument("output", type=Path)
    parser.add_argument("--mode", choices=("missing_tile", "corrupt_tile", "missing_manifest"), required=True)
    args = parser.parse_args()
    if args.output.exists(): raise SystemExit("output already exists")
    shutil.copytree(args.source, args.output)
    manifest = args.output / "manifest.yaml"
    if args.mode == "missing_manifest": manifest.unlink(); return
    tiles = sorted((args.output / "tiles").glob("*.pcd"))
    if not tiles: raise SystemExit("fixture has no tile")
    if args.mode == "missing_tile": tiles[0].unlink()
    else:
        with tiles[0].open("ab") as stream: stream.write(b"CORRUPTED")


if __name__ == "__main__": main()
