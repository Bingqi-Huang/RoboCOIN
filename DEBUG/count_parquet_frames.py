#!/usr/bin/env python3
"""Count frames per episode from parquet files.

Supports:
- A single parquet file
- A directory (recursively scans *.parquet)
- A glob pattern

If no path is passed, the script prompts for one.
"""

from __future__ import annotations

import argparse
import glob
import sys
from collections import Counter, defaultdict
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Count frames in parquet data and summarize frames per episode."
    )
    parser.add_argument(
        "path",
        nargs="?",
        help="Parquet file path, directory, or glob pattern (e.g. data/**/*.parquet).",
    )
    parser.add_argument(
        "--per-file",
        action="store_true",
        help="Also print per-file frame counts.",
    )
    return parser.parse_args()


def prompt_for_path() -> str:
    try:
        raw = input("Enter parquet file/dir/glob path: ").strip()
    except EOFError:
        raise SystemExit("No input received.")

    raw = raw.strip("\"'")
    if not raw:
        raise SystemExit("Empty path.")
    return raw


def resolve_parquet_files(path_text: str) -> list[Path]:
    path_text = path_text.strip()
    path = Path(path_text).expanduser()

    has_glob = any(ch in path_text for ch in "*?[]")
    files: list[Path] = []

    if has_glob:
        files = [Path(p).expanduser() for p in glob.glob(str(path), recursive=True)]
    elif path.is_file():
        files = [path]
    elif path.is_dir():
        files = list(path.rglob("*.parquet"))
    else:
        # Last chance: plain glob (for relative patterns without wildcard chars in some shells)
        files = [Path(p).expanduser() for p in glob.glob(path_text, recursive=True)]

    parquet_files = sorted({p.resolve() for p in files if p.suffix.lower() == ".parquet"})
    return parquet_files


def count_file(file_path: Path) -> tuple[int, Counter]:
    try:
        import pyarrow.parquet as pq
    except ImportError as exc:
        raise SystemExit(
            "Missing dependency: pyarrow. Install it in this environment first."
        ) from exc

    parquet = pq.ParquetFile(file_path)
    rows = parquet.metadata.num_rows if parquet.metadata is not None else 0

    if "episode_index" not in parquet.schema.names:
        return rows, Counter()

    table = parquet.read(columns=["episode_index"])
    values = table.column("episode_index").to_pylist()
    episode_counts = Counter(int(v) for v in values if v is not None)
    return rows, episode_counts


def main() -> int:
    args = parse_args()
    path_text = args.path if args.path else prompt_for_path()
    files = resolve_parquet_files(path_text)

    if not files:
        print(f"No parquet files found for: {path_text}")
        return 1

    total_rows = 0
    per_episode = Counter()
    episode_file_count: dict[int, int] = defaultdict(int)
    per_file_rows: list[tuple[Path, int]] = []
    failed: list[tuple[Path, str]] = []

    for fpath in files:
        try:
            rows, ep_counts = count_file(fpath)
        except Exception as exc:  # noqa: BLE001
            failed.append((fpath, str(exc)))
            continue

        total_rows += rows
        per_file_rows.append((fpath, rows))
        per_episode.update(ep_counts)
        for ep in ep_counts:
            episode_file_count[ep] += 1

    print(f"Scanned {len(files)} parquet file(s).")
    print(f"Total rows (frames): {total_rows}")

    if per_episode:
        print("\nFrames per episode:")
        print(f"{'episode':>10} {'frames':>12} {'files':>8}")
        for ep in sorted(per_episode):
            print(f"{ep:>10} {per_episode[ep]:>12} {episode_file_count[ep]:>8}")
    else:
        print("\nNo 'episode_index' column found; cannot split by episode.")

    if args.per_file:
        print("\nPer-file rows:")
        for fpath, rows in per_file_rows:
            print(f"{rows:>12}  {fpath}")

    if failed:
        print(f"\nFailed to read {len(failed)} file(s):", file=sys.stderr)
        for fpath, err in failed:
            print(f"- {fpath}: {err}", file=sys.stderr)
        return 2

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
