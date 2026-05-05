"""Unit tests for OdomBuffer and TimesyncDiagnostics."""
import sys
sys.path.insert(0, '/opt/ros/humble/lib/python3.10/site-packages')

from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticStatus
from sonar_3d_reconstruction.odom_buffer import OdomBuffer
from sonar_3d_reconstruction.timesync_diagnostics import TimesyncDiagnostics, RollingMean


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
    rm.add(1.0); rm.add(2.0); rm.add(3.0); rm.add(4.0)
    assert abs(rm.value() - 3.0) < 1e-6


def test_diagnostic_msg_warn():
    d = TimesyncDiagnostics()
    d.stamp_diff_mean.add(0.05)
    d.paired_count = 100
    d.dropped_stale_odom = 5
    # Use a minimal node_clock_now stub
    class _Time:
        def to_msg(self):
            from builtin_interfaces.msg import Time as TimeMsg
            return TimeMsg(sec=10, nanosec=0)
    msg = d.to_msg(_Time())
    assert msg.status[0].level == DiagnosticStatus.WARN
    assert any(kv.key == 'paired_count' and kv.value == '100' for kv in msg.status[0].values)
