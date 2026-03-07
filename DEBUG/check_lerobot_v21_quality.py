#!/usr/bin/env python3
"""LeRobot v2.1 dataset quality checker for RoboCOIN.

Checks:
- Dataset structure and metadata integrity (meta/info.json, episodes.jsonl, tasks.jsonl)
- Per-episode parquet correctness (required columns, lengths, indices, timestamps)
- Basic numeric sanity (NaN/Inf checks on core columns and key vectors)
- Video file presence for video-backed camera keys
- Optional quality plots (episode length/fps and per-episode traces)

This script is designed for the LeRobot v2.1 format used by RoboCOIN (based on LeRobot v0.3.4).
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
import pyarrow.parquet as pq


REQUIRED_META_FILES = (
    "meta/info.json",
    "meta/episodes.jsonl",
    "meta/tasks.jsonl",
)

REQUIRED_PARQUET_COLUMNS = (
    "timestamp",
    "frame_index",
    "episode_index",
    "task_index",
    "index",
)


@dataclass
class EpisodeMetrics:
    episode_index: int
    parquet_path: str
    rows: int
    expected_length: int
    frame_index_ok: bool
    episode_index_ok: bool
    index_consecutive_ok: bool
    timestamp_monotonic_ok: bool
    timestamp_dt_mean_ms: float
    timestamp_dt_std_ms: float
    timestamp_dt_p95_ms: float
    timestamp_dt_max_ms: float
    estimated_collection_hz: float | None
    unknown_task_index_count: int
    nan_count_action: int
    nan_count_observation_state: int
    inf_count_action: int
    inf_count_observation_state: int


def _read_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            rows.append(json.loads(line))
    return rows


def _column_to_numpy(column) -> np.ndarray:
    # For scalar columns, to_numpy works; fallback to pylist for compatibility.
    try:
        arr = column.to_numpy(zero_copy_only=False)
        return np.asarray(arr)
    except Exception:
        return np.asarray(column.to_pylist())


def _vector_column_nan_inf_counts(path: Path, key: str) -> tuple[int, int]:
    table = pq.read_table(path, columns=[key])
    values = table[key].to_pylist()
    if len(values) == 0:
        return 0, 0

    try:
        arr = np.asarray(values, dtype=np.float64)
    except Exception:
        # Robust fallback if pyarrow returns nested python objects
        arr = np.asarray([np.asarray(v, dtype=np.float64).ravel() for v in values], dtype=np.float64)

    return int(np.isnan(arr).sum()), int(np.isinf(arr).sum())


def _resolve_dataset_root(path_or_repo: str | None) -> Path:
    if not path_or_repo:
        raw = input("Enter dataset root path or repo_id (e.g. bingqi/name): ").strip().strip("\"'")
    else:
        raw = path_or_repo.strip().strip("\"'")

    candidate = Path(raw).expanduser()
    if candidate.exists():
        if candidate.is_file():
            # If user passed a parquet path, walk up to dataset root
            for parent in [candidate.parent] + list(candidate.parents):
                if (parent / "meta" / "info.json").is_file():
                    return parent
            raise FileNotFoundError(f"Could not locate dataset root from file path: {candidate}")
        if (candidate / "meta" / "info.json").is_file():
            return candidate
        raise FileNotFoundError(f"Path exists but is not a LeRobot dataset root: {candidate}")

    # Try resolving as repo_id under HF cache
    try:
        from lerobot.constants import HF_LEROBOT_HOME
    except Exception as e:
        raise FileNotFoundError(
            f"Path '{raw}' does not exist and repo_id resolution failed ({e})."
        ) from e

    repo_root = HF_LEROBOT_HOME / raw
    if (repo_root / "meta" / "info.json").is_file():
        return repo_root

    raise FileNotFoundError(
        f"Could not resolve dataset root from '{raw}'. "
        f"Tried path and repo cache location: {repo_root}"
    )


def _load_metadata(root: Path) -> tuple[dict[str, Any], dict[int, dict[str, Any]], dict[int, str]]:
    info = _read_json(root / "meta" / "info.json")
    episodes_rows = _read_jsonl(root / "meta" / "episodes.jsonl")
    tasks_rows = _read_jsonl(root / "meta" / "tasks.jsonl")

    episodes = {int(row["episode_index"]): row for row in episodes_rows}
    tasks = {int(row["task_index"]): row["task"] for row in tasks_rows}
    return info, episodes, tasks


def _make_episode_parquet_path(root: Path, info: dict[str, Any], ep_idx: int) -> Path:
    chunks_size = int(info.get("chunks_size", 1000))
    data_tpl = info["data_path"]
    rel = data_tpl.format(episode_chunk=ep_idx // chunks_size, episode_index=ep_idx)
    return root / rel


def _make_episode_video_path(root: Path, info: dict[str, Any], ep_idx: int, video_key: str) -> Path:
    chunks_size = int(info.get("chunks_size", 1000))
    video_tpl = info["video_path"]
    rel = video_tpl.format(episode_chunk=ep_idx // chunks_size, episode_index=ep_idx, video_key=video_key)
    return root / rel


def _choose_plot_episodes(all_eps: list[int], requested: list[int] | None, sample_n: int) -> list[int]:
    if requested:
        return sorted(set(requested))
    if len(all_eps) <= sample_n:
        return list(all_eps)
    # Evenly sample episodes
    idx = np.linspace(0, len(all_eps) - 1, sample_n, dtype=int)
    return sorted({all_eps[i] for i in idx})


def _plot_if_available():
    try:
        import matplotlib.pyplot as plt

        return plt
    except Exception:
        return None


def run_check(
    dataset_root: Path,
    expected_episode_time_s: float | None,
    timestamp_tolerance_s: float,
    plot_episodes: list[int] | None,
    sample_plot_episodes: int,
    out_dir: Path,
    no_plots: bool,
) -> tuple[list[str], list[str], list[EpisodeMetrics], Path]:
    errors: list[str] = []
    warnings: list[str] = []

    # Basic filesystem checks
    for rel in REQUIRED_META_FILES:
        p = dataset_root / rel
        if not p.is_file():
            errors.append(f"Missing required metadata file: {p}")
    if errors:
        return errors, warnings, [], out_dir

    info, episodes_meta, tasks = _load_metadata(dataset_root)

    # Version compatibility check
    codebase_version = str(info.get("codebase_version", "unknown"))
    if not codebase_version.startswith("v2.1"):
        warnings.append(
            f"Dataset codebase_version is '{codebase_version}', expected v2.1 for current RoboCOIN support."
        )

    features: dict[str, dict] = info.get("features", {})
    video_keys = [k for k, ft in features.items() if ft.get("dtype") == "video"]
    target_fps = float(info.get("fps", 0))
    if target_fps <= 0:
        errors.append(f"Invalid fps in meta/info.json: {target_fps}")

    if int(info.get("total_episodes", -1)) != len(episodes_meta):
        warnings.append(
            f"meta/info.json total_episodes={info.get('total_episodes')} but episodes.jsonl has {len(episodes_meta)}."
        )

    episode_indices = sorted(episodes_meta.keys())
    metrics: list[EpisodeMetrics] = []
    global_index_seen: list[tuple[int, int, int]] = []  # (episode, first_index, last_index)

    for ep_idx in episode_indices:
        ep_meta = episodes_meta[ep_idx]
        exp_len = int(ep_meta.get("length", -1))
        parquet_path = _make_episode_parquet_path(dataset_root, info, ep_idx)

        if not parquet_path.is_file():
            errors.append(f"Episode {ep_idx}: missing parquet file: {parquet_path}")
            continue

        pf = pq.ParquetFile(parquet_path)
        schema_names = set(pf.schema.names)
        missing_req = [c for c in REQUIRED_PARQUET_COLUMNS if c not in schema_names]
        if missing_req:
            errors.append(f"Episode {ep_idx}: missing required parquet columns: {missing_req}")
            continue

        # Read core scalar columns
        table = pq.read_table(parquet_path, columns=list(REQUIRED_PARQUET_COLUMNS))
        ts = _column_to_numpy(table["timestamp"]).astype(np.float64)
        frame_idx = _column_to_numpy(table["frame_index"]).astype(np.int64)
        ep_col = _column_to_numpy(table["episode_index"]).astype(np.int64)
        task_idx = _column_to_numpy(table["task_index"]).astype(np.int64)
        global_idx = _column_to_numpy(table["index"]).astype(np.int64)
        rows = len(frame_idx)

        if rows != exp_len:
            errors.append(f"Episode {ep_idx}: parquet rows={rows} != metadata length={exp_len}.")

        frame_ok = bool(np.array_equal(frame_idx, np.arange(rows, dtype=np.int64)))
        if not frame_ok:
            errors.append(f"Episode {ep_idx}: frame_index is not contiguous from 0..N-1.")

        episode_ok = bool(np.all(ep_col == ep_idx))
        if not episode_ok:
            errors.append(f"Episode {ep_idx}: episode_index column contains mismatched values.")

        index_ok = bool(rows <= 1 or np.all(np.diff(global_idx) == 1))
        if not index_ok:
            warnings.append(f"Episode {ep_idx}: global 'index' is not strictly consecutive.")

        if rows > 0:
            global_index_seen.append((ep_idx, int(global_idx[0]), int(global_idx[-1])))

        ts_mono_ok = bool(rows <= 1 or np.all(np.diff(ts) > 0))
        if not ts_mono_ok:
            errors.append(f"Episode {ep_idx}: timestamp is not strictly increasing.")

        if rows > 1:
            dt = np.diff(ts)
            dt_mean_ms = float(dt.mean() * 1000.0)
            dt_std_ms = float(dt.std() * 1000.0)
            dt_p95_ms = float(np.percentile(dt, 95) * 1000.0)
            dt_max_ms = float(dt.max() * 1000.0)
            target_dt = 1.0 / target_fps if target_fps > 0 else math.nan
            bad_dt = np.abs(dt - target_dt) > timestamp_tolerance_s
            if np.any(bad_dt):
                bad_ratio = float(np.mean(bad_dt))
                warnings.append(
                    f"Episode {ep_idx}: {bad_ratio * 100:.2f}% timestamp diffs exceed tolerance "
                    f"{timestamp_tolerance_s * 1000:.3f}ms."
                )
        else:
            dt_mean_ms = 0.0
            dt_std_ms = 0.0
            dt_p95_ms = 0.0
            dt_max_ms = 0.0

        unknown_task_count = int(np.sum(~np.isin(task_idx, list(tasks.keys()))))
        if unknown_task_count > 0:
            errors.append(f"Episode {ep_idx}: found {unknown_task_count} unknown task_index values.")

        nan_action = inf_action = nan_obs = inf_obs = 0
        if "action" in schema_names:
            nan_action, inf_action = _vector_column_nan_inf_counts(parquet_path, "action")
            if nan_action > 0 or inf_action > 0:
                errors.append(
                    f"Episode {ep_idx}: action has nan={nan_action}, inf={inf_action}."
                )
        if "observation.state" in schema_names:
            nan_obs, inf_obs = _vector_column_nan_inf_counts(parquet_path, "observation.state")
            if nan_obs > 0 or inf_obs > 0:
                errors.append(
                    f"Episode {ep_idx}: observation.state has nan={nan_obs}, inf={inf_obs}."
                )

        est_hz = None
        if expected_episode_time_s and expected_episode_time_s > 0:
            est_hz = rows / expected_episode_time_s

        metrics.append(
            EpisodeMetrics(
                episode_index=ep_idx,
                parquet_path=str(parquet_path),
                rows=rows,
                expected_length=exp_len,
                frame_index_ok=frame_ok,
                episode_index_ok=episode_ok,
                index_consecutive_ok=index_ok,
                timestamp_monotonic_ok=ts_mono_ok,
                timestamp_dt_mean_ms=dt_mean_ms,
                timestamp_dt_std_ms=dt_std_ms,
                timestamp_dt_p95_ms=dt_p95_ms,
                timestamp_dt_max_ms=dt_max_ms,
                estimated_collection_hz=est_hz,
                unknown_task_index_count=unknown_task_count,
                nan_count_action=nan_action,
                nan_count_observation_state=nan_obs,
                inf_count_action=inf_action,
                inf_count_observation_state=inf_obs,
            )
        )

    # Global index continuity across episodes (if present)
    if global_index_seen:
        global_index_seen.sort(key=lambda x: x[1])
        for i in range(len(global_index_seen) - 1):
            ep_a, _, last_a = global_index_seen[i]
            ep_b, first_b, _ = global_index_seen[i + 1]
            if first_b != last_a + 1:
                warnings.append(
                    f"Global index gap/overlap between episode {ep_a} and {ep_b}: {last_a} -> {first_b}"
                )

    # Video existence checks
    if info.get("video_path") and video_keys:
        missing_video_files = 0
        for ep_idx in episode_indices:
            for vid_key in video_keys:
                vpath = _make_episode_video_path(dataset_root, info, ep_idx, vid_key)
                if not vpath.is_file():
                    missing_video_files += 1
        if missing_video_files > 0:
            warnings.append(f"Missing {missing_video_files} expected video files.")

    # Cross-check total frames
    frame_sum = int(sum(m.rows for m in metrics))
    meta_total_frames = int(info.get("total_frames", -1))
    if meta_total_frames != frame_sum:
        warnings.append(f"meta total_frames={meta_total_frames}, but summed parquet rows={frame_sum}.")

    # Save report artifacts
    out_dir.mkdir(parents=True, exist_ok=True)
    report = {
        "dataset_root": str(dataset_root),
        "codebase_version": codebase_version,
        "target_fps": target_fps,
        "total_episodes_meta": int(info.get("total_episodes", -1)),
        "total_frames_meta": meta_total_frames,
        "total_frames_from_parquet": frame_sum,
        "errors": errors,
        "warnings": warnings,
        "generated_at": datetime.now().isoformat(timespec="seconds"),
    }
    (out_dir / "summary.json").write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")

    with (out_dir / "episode_metrics.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(asdict(metrics[0]).keys()) if metrics else [])
        if metrics:
            writer.writeheader()
            for m in metrics:
                writer.writerow(asdict(m))

    # Plots
    if not no_plots and metrics:
        plt = _plot_if_available()
        if plt is None:
            warnings.append("matplotlib is unavailable; skipping plots.")
        else:
            ep = np.array([m.episode_index for m in metrics], dtype=int)
            rows = np.array([m.rows for m in metrics], dtype=float)

            fig, ax = plt.subplots(figsize=(10, 4))
            ax.plot(ep, rows, marker="o", linewidth=1.5)
            ax.set_title("Frames Per Episode")
            ax.set_xlabel("Episode Index")
            ax.set_ylabel("Frames")
            ax.grid(True, alpha=0.3)
            fig.tight_layout()
            fig.savefig(out_dir / "frames_per_episode.png", dpi=140)
            plt.close(fig)

            if expected_episode_time_s and expected_episode_time_s > 0:
                hz = rows / expected_episode_time_s
                fig, ax = plt.subplots(figsize=(10, 4))
                ax.plot(ep, hz, marker="o", linewidth=1.5, label="Estimated collection Hz")
                if target_fps > 0:
                    ax.axhline(target_fps, color="r", linestyle="--", linewidth=1.2, label="Target fps")
                ax.set_title("Estimated Collection Rate (rows / expected_episode_time_s)")
                ax.set_xlabel("Episode Index")
                ax.set_ylabel("Hz")
                ax.grid(True, alpha=0.3)
                ax.legend()
                fig.tight_layout()
                fig.savefig(out_dir / "estimated_collection_hz.png", dpi=140)
                plt.close(fig)

            plot_eps = _choose_plot_episodes([m.episode_index for m in metrics], plot_episodes, sample_plot_episodes)
            for ep_idx in plot_eps:
                parquet_path = _make_episode_parquet_path(dataset_root, info, ep_idx)
                schema_names = set(pq.ParquetFile(parquet_path).schema.names)
                cols = ["frame_index", "timestamp"]
                if "observation.state" in schema_names:
                    cols.append("observation.state")
                if "action" in schema_names:
                    cols.append("action")
                table = pq.read_table(parquet_path, columns=cols)
                frame_idx = _column_to_numpy(table["frame_index"]).astype(np.int64)
                ts = _column_to_numpy(table["timestamp"]).astype(np.float64)

                nrows = 1
                if "observation.state" in cols:
                    nrows += 1
                if "action" in cols:
                    nrows += 1

                fig, axes = plt.subplots(nrows=nrows, ncols=1, figsize=(11, 3.0 * nrows), sharex=True)
                if nrows == 1:
                    axes = [axes]

                ax0 = axes[0]
                if len(ts) > 1:
                    dt_ms = np.diff(ts) * 1000.0
                    ax0.plot(frame_idx[1:], dt_ms, linewidth=1.0)
                    if target_fps > 0:
                        ax0.axhline((1.0 / target_fps) * 1000.0, color="r", linestyle="--", linewidth=1.0)
                    ax0.set_ylabel("dt (ms)")
                    ax0.set_title(f"Episode {ep_idx}: Timestamp Delta")
                    ax0.grid(True, alpha=0.3)
                else:
                    ax0.text(0.5, 0.5, "Not enough frames", ha="center", va="center", transform=ax0.transAxes)

                row_id = 1
                for key in ("observation.state", "action"):
                    if key not in cols:
                        continue
                    arr = np.asarray(table[key].to_pylist(), dtype=np.float64)
                    if arr.ndim == 1:
                        arr = arr[:, None]
                    dims = min(arr.shape[1], 8)
                    ax = axes[row_id]
                    for d in range(dims):
                        ax.plot(frame_idx, arr[:, d], linewidth=0.9, label=f"d{d}")
                    ax.set_ylabel(key)
                    ax.grid(True, alpha=0.3)
                    if dims <= 6:
                        ax.legend(loc="upper right", fontsize=8, ncol=2)
                    row_id += 1

                axes[-1].set_xlabel("frame_index")
                fig.tight_layout()
                fig.savefig(out_dir / f"episode_{ep_idx:06d}_traces.png", dpi=140)
                plt.close(fig)

    return errors, warnings, metrics, out_dir


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Check LeRobot v2.1 dataset quality/correctness and generate plots from parquet."
    )
    parser.add_argument(
        "dataset",
        nargs="?",
        help="Dataset root path, parquet path, or repo_id (e.g. bingqi/rm75_xxx).",
    )
    parser.add_argument(
        "--expected-episode-time-s",
        type=float,
        default=None,
        help="Expected wall-clock episode duration in seconds (for estimated collection Hz).",
    )
    parser.add_argument(
        "--timestamp-tolerance-s",
        type=float,
        default=1e-4,
        help="Tolerance for timestamp step check against 1/fps.",
    )
    parser.add_argument(
        "--episode",
        type=int,
        nargs="*",
        default=None,
        help="Episode indices to generate detailed trace plots for.",
    )
    parser.add_argument(
        "--sample-plot-episodes",
        type=int,
        default=3,
        help="If --episode is not provided, number of episodes to sample for trace plots.",
    )
    parser.add_argument(
        "--out-dir",
        type=str,
        default=None,
        help="Output directory for report/plots. Default: DEBUG/dataset_quality_report_<timestamp>.",
    )
    parser.add_argument(
        "--no-plots",
        action="store_true",
        help="Skip matplotlib plots and only generate textual report.",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Return non-zero exit code also when warnings are present.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    try:
        root = _resolve_dataset_root(args.dataset)
    except Exception as e:
        print(f"[ERROR] {e}")
        return 2

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = Path(args.out_dir) if args.out_dir else Path("DEBUG") / f"dataset_quality_report_{ts}"

    errors, warnings, metrics, saved_dir = run_check(
        dataset_root=root,
        expected_episode_time_s=args.expected_episode_time_s,
        timestamp_tolerance_s=args.timestamp_tolerance_s,
        plot_episodes=args.episode,
        sample_plot_episodes=args.sample_plot_episodes,
        out_dir=out_dir,
        no_plots=args.no_plots,
    )

    print("\n=== LeRobot v2.1 Dataset Quality Report ===")
    print(f"Dataset root: {root}")
    print(f"Episodes checked: {len(metrics)}")
    if metrics:
        frames = np.array([m.rows for m in metrics], dtype=float)
        print(
            f"Frames/episode: min={int(frames.min())}, max={int(frames.max())}, "
            f"mean={frames.mean():.2f}, std={frames.std():.2f}"
        )

    if warnings:
        print(f"\n[WARNINGS] {len(warnings)}")
        for w in warnings:
            print(f"- {w}")
    else:
        print("\n[WARNINGS] 0")

    if errors:
        print(f"\n[ERRORS] {len(errors)}")
        for e in errors:
            print(f"- {e}")
    else:
        print("\n[ERRORS] 0")

    print(f"\nSaved report to: {saved_dir}")
    print(f"- summary: {saved_dir / 'summary.json'}")
    print(f"- metrics: {saved_dir / 'episode_metrics.csv'}")

    if errors:
        return 2
    if args.strict and warnings:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

