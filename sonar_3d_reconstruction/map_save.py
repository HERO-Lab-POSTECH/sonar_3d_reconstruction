"""Workspace-standard map save utility (spec §2.9)."""
import os
import re
from datetime import datetime
from pathlib import Path


def get_default_map_dir(pkg_name: str) -> Path:
    """`$PKRC_MAP_DIR/<pkg>` or `$HOME/data/maps/<pkg>`."""
    base = os.environ.get('PKRC_MAP_DIR', os.path.expanduser('~/data/maps'))
    return Path(base) / pkg_name


def resolve_map_save_path(
    user_path: str, pkg_name: str, map_filename: str
) -> Path:
    """Empty user_path → auto-timestamp dir; else use as-is.

    Returns the final file path (parent dirs created).
    """
    if user_path:
        full = Path(user_path)
        full.parent.mkdir(parents=True, exist_ok=True)
        return full
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    target_dir = get_default_map_dir(pkg_name) / ts
    target_dir.mkdir(parents=True, exist_ok=True)
    return target_dir / map_filename


def update_latest_symlink(saved_path: Path, pkg_name: str) -> None:
    """Update `<map_dir>/<pkg>/latest -> <ts>` (relative symlink).

    No-op if saved_path's parent isn't a timestamp directory (i.e., user
    supplied an explicit path).
    """
    ts_dir = saved_path.parent
    if not ts_dir.is_dir() or not re.match(r'^\d{8}_\d{6}$', ts_dir.name):
        return
    latest_link = get_default_map_dir(pkg_name) / 'latest'
    tmp_link = latest_link.parent / (latest_link.name + '.tmp')
    if tmp_link.is_symlink() or tmp_link.exists():
        tmp_link.unlink()
    tmp_link.symlink_to(ts_dir.name)
    os.replace(str(tmp_link), str(latest_link))
