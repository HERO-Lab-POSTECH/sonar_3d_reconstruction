"""Ring buffer for odometry messages with interpolate/nearest selection."""
import math
import threading
from collections import deque
from typing import Optional

from builtin_interfaces.msg import Time as TimeMsg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


def _stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _sec_to_stamp(target_sec: float) -> TimeMsg:
    """Convert float seconds to builtin_interfaces/Time without precision loss
    near the integer boundary."""
    sec = int(math.floor(target_sec))
    nanosec = int(round((target_sec - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return TimeMsg(sec=sec, nanosec=nanosec)


def _slerp_quaternion(q0: Quaternion, q1: Quaternion, t: float) -> Quaternion:
    """SLERP between two unit quaternions."""
    dot = q0.x*q1.x + q0.y*q1.y + q0.z*q1.z + q0.w*q1.w
    if dot < 0.0:
        q1 = Quaternion(x=-q1.x, y=-q1.y, z=-q1.z, w=-q1.w)
        dot = -dot
    if dot > 0.9995:
        out = Quaternion(
            x=q0.x + t * (q1.x - q0.x),
            y=q0.y + t * (q1.y - q0.y),
            z=q0.z + t * (q1.z - q0.z),
            w=q0.w + t * (q1.w - q0.w),
        )
        norm = math.sqrt(out.x**2 + out.y**2 + out.z**2 + out.w**2)
        return Quaternion(x=out.x/norm, y=out.y/norm, z=out.z/norm, w=out.w/norm)
    theta_0 = math.acos(dot)
    theta = theta_0 * t
    sin_theta = math.sin(theta)
    sin_theta_0 = math.sin(theta_0)
    s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return Quaternion(
        x=s0*q0.x + s1*q1.x,
        y=s0*q0.y + s1*q1.y,
        z=s0*q0.z + s1*q1.z,
        w=s0*q0.w + s1*q1.w,
    )


class OdomBuffer:
    """Thread-safe ring buffer for nav_msgs/Odometry."""

    def __init__(self, max_size: int = 50):
        self._buffer: deque = deque(maxlen=max_size)
        self._lock = threading.Lock()

    def push(self, msg: Odometry) -> None:
        with self._lock:
            self._buffer.append(msg)

    def latest(self) -> Optional[Odometry]:
        with self._lock:
            return self._buffer[-1] if self._buffer else None

    def nearest(self, target_sec: float) -> Optional[Odometry]:
        with self._lock:
            if not self._buffer:
                return None
            return min(
                self._buffer,
                key=lambda m: abs(_stamp_to_sec(m.header.stamp) - target_sec))

    def interpolate(self, target_sec: float) -> Optional[Odometry]:
        with self._lock:
            if len(self._buffer) < 2:
                return None
            stamps = [_stamp_to_sec(m.header.stamp) for m in self._buffer]
            for i in range(len(stamps) - 1):
                if stamps[i] <= target_sec <= stamps[i+1]:
                    t = (target_sec - stamps[i]) / (stamps[i+1] - stamps[i])
                    m0, m1 = self._buffer[i], self._buffer[i+1]
                    out = Odometry()
                    # Stamp the interpolated message at target_sec, not at the
                    # upper bracket — otherwise stamp_diff diagnostics report
                    # |sonar_t - upper_bracket| instead of ~0.
                    out.header.stamp = _sec_to_stamp(target_sec)
                    out.header.frame_id = m0.header.frame_id
                    out.child_frame_id = m0.child_frame_id
                    p0, p1 = m0.pose.pose.position, m1.pose.pose.position
                    out.pose.pose.position.x = p0.x + t * (p1.x - p0.x)
                    out.pose.pose.position.y = p0.y + t * (p1.y - p0.y)
                    out.pose.pose.position.z = p0.z + t * (p1.z - p0.z)
                    out.pose.pose.orientation = _slerp_quaternion(
                        m0.pose.pose.orientation, m1.pose.pose.orientation, t)
                    tw0, tw1 = m0.twist.twist, m1.twist.twist
                    out.twist.twist.linear.x = tw0.linear.x + t * (tw1.linear.x - tw0.linear.x)
                    out.twist.twist.linear.y = tw0.linear.y + t * (tw1.linear.y - tw0.linear.y)
                    out.twist.twist.linear.z = tw0.linear.z + t * (tw1.linear.z - tw0.linear.z)
                    out.twist.twist.angular.x = tw0.angular.x + t * (tw1.angular.x - tw0.angular.x)
                    out.twist.twist.angular.y = tw0.angular.y + t * (tw1.angular.y - tw0.angular.y)
                    out.twist.twist.angular.z = tw0.angular.z + t * (tw1.angular.z - tw0.angular.z)
                    return out
            return None
