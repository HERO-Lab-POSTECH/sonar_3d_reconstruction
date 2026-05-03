#!/usr/bin/env python3
"""Phase B-1.0 회귀 비교 driver.

baseline / candidate 두 bag 의 마지막 PointCloud2 frame 을 읽어
voxel set + log-odds 통계를 비교하고 임계 통과 여부를 판정한다.

Exit code: PASS=0, FAIL=1.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from regression import regression_metric as rm  # noqa: E402


SEPARATOR = "=" * 64


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True,
                        help="baseline bag directory")
    parser.add_argument("--candidate", type=Path, required=True,
                        help="candidate bag directory")
    parser.add_argument("--topic", type=str,
                        default="/pkrc/sonar/cpp_pointcloud",
                        help="PointCloud2 topic to compare")
    parser.add_argument("--resolution", type=float,
                        default=rm.VOXEL_RESOLUTION_DEFAULT,
                        help="voxel resolution in meters")
    parser.add_argument("--jaccard-threshold", type=float, default=1.0,
                        help="minimum Jaccard index for PASS")
    parser.add_argument("--mean-log-odds-threshold", type=float, default=0.0,
                        help="maximum mean |log-odds diff| for PASS")
    parser.add_argument("--proc-time-topic", type=str,
                        default="/pkrc/sonar/processing_time",
                        help="std_msgs/Float64 processing-time topic")
    return parser.parse_args()


def _voxel_set(keys):
    return {(int(k[0]), int(k[1]), int(k[2])) for k in keys}


def _print_proc_time(label: str, stats: dict) -> None:
    if not stats:
        print(f"  {label:<10} proc time: <none>")
        return
    print(
        f"  {label:<10} proc time: count={stats['count']} "
        f"mean={stats['mean']*1e3:.2f}ms p95={stats['p95']*1e3:.2f}ms "
        f"max={stats['max']*1e3:.2f}ms"
    )


def _print_report(args, base, cand, jacc, diff, stats_b, stats_c) -> None:
    print(SEPARATOR)
    print(f"baseline  bag: {args.baseline}")
    print(f"candidate bag: {args.candidate}")
    print(f"topic={args.topic}  resolution={args.resolution}m")
    print(SEPARATOR)
    print(f"  baseline  voxels: {len(base)}")
    print(f"  candidate voxels: {len(cand)}")
    print(f"  jaccard         : {jacc:.6f}")
    print(f"  common voxels   : {diff['common_count']}")
    mean_d = diff['mean_diff']
    max_d = diff['max_diff']
    mean_s = "nan" if math.isnan(mean_d) else f"{mean_d:.6f}"
    max_s = "nan" if math.isnan(max_d) else f"{max_d:.6f}"
    print(f"  mean log-odds Δ : {mean_s}")
    print(f"  max  log-odds Δ : {max_s}")
    _print_proc_time("baseline", stats_b)
    _print_proc_time("candidate", stats_c)
    print(SEPARATOR)


def _safe_read(label: str, bag_dir: Path, topic: str):
    try:
        return rm.read_last_pointcloud(bag_dir, topic)
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        print(f"[compare] FAIL — {label} bag unreadable: {exc}",
              file=sys.stderr)
        return None


def main() -> int:
    args = _parse_args()
    base_frame = _safe_read("baseline", args.baseline, args.topic)
    cand_frame = _safe_read("candidate", args.candidate, args.topic)
    if base_frame is None or cand_frame is None:
        print("[compare] FAIL — missing PointCloud2 frame "
              f"(baseline={base_frame is not None}, "
              f"candidate={cand_frame is not None})", file=sys.stderr)
        return 1

    keys_b = rm.voxelize(base_frame.points, args.resolution)
    keys_c = rm.voxelize(cand_frame.points, args.resolution)
    set_b = _voxel_set(keys_b)
    set_c = _voxel_set(keys_c)

    jacc = rm.jaccard(set_b, set_c)
    diff = rm.log_odds_diff(
        keys_b, base_frame.intensities, keys_c, cand_frame.intensities,
    )
    try:
        stats_b = rm.read_processing_stats(args.baseline, args.proc_time_topic)
        stats_c = rm.read_processing_stats(args.candidate, args.proc_time_topic)
    except Exception:  # noqa: BLE001
        stats_b, stats_c = {}, {}

    _print_report(args, set_b, set_c, jacc, diff, stats_b, stats_c)

    j_pass = jacc >= args.jaccard_threshold
    diff_pass = (
        diff["common_count"] > 0
        and diff["mean_diff"] <= args.mean_log_odds_threshold + 1e-9
    )
    overall = j_pass and diff_pass
    verdict = "PASS" if overall else "FAIL"
    mean_disp = "nan" if math.isnan(diff["mean_diff"]) else f"{diff['mean_diff']:.6f}"
    print(f"[compare] {verdict} (jaccard={jacc:.6f}, log_odds_diff={mean_disp})")
    return 0 if overall else 1


if __name__ == "__main__":
    sys.exit(main())
