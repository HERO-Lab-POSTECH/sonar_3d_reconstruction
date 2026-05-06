"""Unit tests for OdomBuffer and TimesyncDiagnostics."""
import sys

sys.path.insert(0, '/opt/ros/humble/lib/python3.10/site-packages')

from nav_msgs.msg import Odometry  # noqa: E402
from diagnostic_msgs.msg import DiagnosticStatus  # noqa: E402
from sonar_3d_reconstruction.odom_buffer import OdomBuffer  # noqa: E402
from sonar_3d_reconstruction.timesync_diagnostics import (  # noqa: E402
    TimesyncDiagnostics, RollingMean, RollingMax,
)


def _odom(t_sec: float, x: float = 0.0) -> Odometry:
    msg = Odometry()
    msg.header.stamp.sec = int(t_sec)
    msg.header.stamp.nanosec = int((t_sec - int(t_sec)) * 1e9)
    msg.pose.pose.position.x = x
    msg.pose.pose.orientation.w = 1.0
    return msg


def test_latest_empty():
    b = OdomBuffer()
    assert b.latest() is None


def test_latest_returns_last():
    b = OdomBuffer()
    b.push(_odom(1.0))
    b.push(_odom(2.0))
    assert b.latest().header.stamp.sec == 2


def test_nearest():
    b = OdomBuffer()
    for t in [1.0, 1.5, 2.0]:
        b.push(_odom(t))
    n = b.nearest(1.7)
    assert n.header.stamp.sec == 1 and n.header.stamp.nanosec == 500_000_000


def test_interpolate_position():
    b = OdomBuffer()
    b.push(_odom(1.0, x=10.0))
    b.push(_odom(2.0, x=20.0))
    interp = b.interpolate(1.5)
    assert abs(interp.pose.pose.position.x - 15.0) < 1e-6


def test_interpolate_outside_returns_none():
    b = OdomBuffer()
    b.push(_odom(1.0))
    b.push(_odom(2.0))
    assert b.interpolate(0.5) is None
    assert b.interpolate(3.0) is None


def test_rolling_mean():
    rm = RollingMean(3)
    rm.add(1.0)
    rm.add(2.0)
    rm.add(3.0)
    rm.add(4.0)
    assert abs(rm.value() - 3.0) < 1e-6


def test_rolling_max():
    rm = RollingMax(3)
    rm.add(1.0)
    rm.add(3.0)
    rm.add(2.0)
    rm.add(4.0)
    assert abs(rm.value() - 4.0) < 1e-6


class _Time:
    """Minimal clock stub for diagnostics tests."""

    def to_msg(self):
        from builtin_interfaces.msg import Time as TimeMsg
        return TimeMsg(sec=10, nanosec=0)


def test_diagnostic_msg_stale_odom_does_not_gate_level():
    """B-4 option (c): dropped_stale_odom is structurally unreachable
    under default config (max_stamp_diff < max_odom_age). The counter
    is exposed for visibility but no longer gates the diagnostic level.
    """
    d = TimesyncDiagnostics()
    d.stamp_diff_mean.add(0.05)
    d.paired_count = 100
    d.dropped_stale_odom = 5
    msg = d.to_msg(_Time())
    assert msg.status[0].level == DiagnosticStatus.OK
    assert any(
        kv.key == 'paired_count' and kv.value == '100'
        for kv in msg.status[0].values
    )
    assert any(
        kv.key == 'dropped_stale_odom' and kv.value == '5'
        for kv in msg.status[0].values
    )


def test_diagnostic_msg_warn_on_quality_gate_accumulation():
    """Sustained SLAM quality-gate drops (>10) promote to WARN."""
    d = TimesyncDiagnostics()
    d.dropped_quality_gate = 11
    msg = d.to_msg(_Time())
    assert msg.status[0].level == DiagnosticStatus.WARN


def test_diagnostic_msg_warn_on_stamp_diff():
    d = TimesyncDiagnostics()
    d.stamp_diff_max.add(0.2)  # exceeds default 0.1 threshold
    msg = d.to_msg(_Time())
    assert msg.status[0].level == DiagnosticStatus.WARN


def test_diagnostic_msg_ok():
    d = TimesyncDiagnostics()
    d.stamp_diff_max.add(0.05)  # within threshold
    msg = d.to_msg(_Time())
    assert msg.status[0].level == DiagnosticStatus.OK
