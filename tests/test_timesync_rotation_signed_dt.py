"""Verify rotation compensation uses signed dt (not abs) so that
future-stamped odom causes backward extrapolation, not forward."""
from copy import deepcopy
from nav_msgs.msg import Odometry


def _make_odom(yaw_rate: float) -> Odometry:
    msg = Odometry()
    msg.twist.twist.angular.z = yaw_rate
    msg.pose.pose.orientation.w = 1.0
    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = 0.0
    return msg


def _apply(yaw_rate: float, dt: float) -> Odometry:
    """Reproduce the small-angle approx in 3d_mapper_node._apply_rotation_compensation."""
    import math
    msg = _make_odom(yaw_rate)
    out = deepcopy(msg)
    half_dt = dt * 0.5
    qd_x = msg.twist.twist.angular.x * half_dt
    qd_y = msg.twist.twist.angular.y * half_dt
    qd_z = msg.twist.twist.angular.z * half_dt
    qd_w = 1.0
    norm = math.sqrt(qd_x**2 + qd_y**2 + qd_z**2 + qd_w**2)
    qd_x, qd_y, qd_z, qd_w = qd_x/norm, qd_y/norm, qd_z/norm, qd_w/norm
    q = msg.pose.pose.orientation
    out.pose.pose.orientation.w = q.w*qd_w - q.x*qd_x - q.y*qd_y - q.z*qd_z
    out.pose.pose.orientation.z = q.w*qd_z + q.x*qd_y - q.y*qd_x + q.z*qd_w
    return out


def test_signed_dt_forward_for_past_odom():
    """sonar_t > odom_t (odom is older): dt > 0, qz > 0 for positive yaw_rate."""
    out = _apply(yaw_rate=1.0, dt=0.1)  # signed positive
    assert out.pose.pose.orientation.z > 0.0


def test_signed_dt_backward_for_future_odom():
    """sonar_t < odom_t (odom is newer/future): dt < 0, qz < 0 for positive yaw_rate.
    PR-A C-2 abs() bug forced this case to qz > 0 -- wrong direction."""
    out = _apply(yaw_rate=1.0, dt=-0.1)  # signed negative
    assert out.pose.pose.orientation.z < 0.0
