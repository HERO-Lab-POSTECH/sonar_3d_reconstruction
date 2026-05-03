#!/usr/bin/env python3
"""Phase B-1.0 회귀 시각화 driver.

baseline / candidate 두 bag 의 마지막 PointCloud2 frame 을 읽어
XY top-down + XZ side-view scatter 비교 PNG 를 저장한다.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from regression import regression_metric as rm  # noqa: E402

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError as exc:
    print(f"[plot] matplotlib import failed: {exc}", file=sys.stderr)
    print("[plot] install via: pip install matplotlib", file=sys.stderr)
    sys.exit(2)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--baseline", type=Path,
        default=Path("/tmp/sonar3d_regression/baseline/cloud"),
        help="baseline bag directory",
    )
    parser.add_argument(
        "--candidate", type=Path,
        default=Path("/tmp/sonar3d_regression/candidate/cloud"),
        help="candidate bag directory",
    )
    parser.add_argument("--topic", type=str,
                        default="/pkrc/sonar/cpp_pointcloud",
                        help="PointCloud2 topic to plot")
    parser.add_argument(
        "--out", type=Path,
        default=Path("/tmp/sonar3d_regression/comparison.png"),
        help="output PNG path",
    )
    return parser.parse_args()


def _scatter(ax, pts_b, pts_c, idx_x, idx_y, title, xlabel, ylabel):
    if pts_b is not None and pts_b.shape[0] > 0:
        ax.scatter(pts_b[:, idx_x], pts_b[:, idx_y],
                   s=1.0, alpha=0.4, c="tab:blue", label="baseline")
    if pts_c is not None and pts_c.shape[0] > 0:
        ax.scatter(pts_c[:, idx_x], pts_c[:, idx_y],
                   s=1.0, alpha=0.4, c="tab:orange", label="candidate")
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", markerscale=4)


def _safe_read(label: str, bag_dir: Path, topic: str):
    try:
        return rm.read_last_pointcloud(bag_dir, topic)
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        print(f"[plot] {label} bag unreadable: {exc}", file=sys.stderr)
        return None


def main() -> int:
    args = _parse_args()
    base_frame = _safe_read("baseline", args.baseline, args.topic)
    cand_frame = _safe_read("candidate", args.candidate, args.topic)
    if base_frame is None and cand_frame is None:
        print("[plot] FAIL — both baseline and candidate are empty",
              file=sys.stderr)
        return 1

    pts_b = base_frame.points if base_frame is not None else None
    pts_c = cand_frame.points if cand_frame is not None else None
    n_b = 0 if pts_b is None else pts_b.shape[0]
    n_c = 0 if pts_c is None else pts_c.shape[0]

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    _scatter(axes[0], pts_b, pts_c, 0, 1,
             "XY top-down", "x [m]", "y [m]")
    axes[0].set_aspect("equal", adjustable="datalim")
    _scatter(axes[1], pts_b, pts_c, 0, 2,
             "XZ side view", "x [m]", "z [m]")

    fig.suptitle(
        "sonar3d regression — Phase B-1\n"
        f"baseline={n_b} pts, candidate={n_c} pts"
    )
    fig.tight_layout(rect=(0, 0, 1, 0.94))

    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=120)
    plt.close(fig)
    print(f"[plot] saved {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
