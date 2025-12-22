#!/usr/bin/env python3
"""
World Init TF Broadcaster Node

Uses IMU acceleration data to calculate initial horizontal plane and
broadcasts world_init -> camera_init Static TF.

TF structure:
  world_init (horizontal plane) -> camera_init (tilted initial pose) -> body

Author: Sonar 3D Reconstruction Team
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


def euler_from_rotation_matrix(R):
    """Extract roll, pitch, yaw from rotation matrix (ZYX convention)"""
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = sy < 1e-6
    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0
    return roll, pitch, yaw


def quaternion_from_euler(roll, pitch, yaw):
    """Convert euler angles to quaternion (ZYX convention)"""
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return np.array([qx, qy, qz, qw])


def quaternion_inverse(q):
    """Inverse of quaternion [x, y, z, w]"""
    return np.array([-q[0], -q[1], -q[2], q[3]])


class WorldInitBroadcaster(Node):
    def __init__(self):
        super().__init__('world_init_broadcaster')

        # Parameters
        self.declare_parameter('imu_topic', '/sensor/ins/livox_mid360/imu')
        self.declare_parameter('init_samples', 50)  # Number of IMU samples
        self.declare_parameter('parent_frame', 'world_init')
        self.declare_parameter('child_frame', 'camera_init')

        imu_topic = self.get_parameter('imu_topic').value
        self.init_samples = self.get_parameter('init_samples').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value

        # State
        self.acc_samples = []
        self.initialized = False

        # TF broadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # QoS
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # IMU subscriber
        self.imu_sub = self.create_subscription(
            Imu, imu_topic, self.imu_callback, qos
        )

        self.get_logger().info(f'Waiting for {self.init_samples} IMU samples...')

    def imu_callback(self, msg: Imu):
        if self.initialized:
            return

        # Collect acceleration samples
        acc = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        self.acc_samples.append(acc)

        if len(self.acc_samples) >= self.init_samples:
            self.compute_and_publish_tf()

    def compute_and_publish_tf(self):
        """Compute gravity alignment and publish TF"""
        # Average acceleration
        mean_acc = np.mean(self.acc_samples, axis=0)

        # Measured gravity direction (normalized)
        g_measured = mean_acc / np.linalg.norm(mean_acc)

        # Reference gravity (z-down in IMU frame when horizontal)
        # IMU convention: when horizontal, acc = [0, 0, +g] (measures reaction to gravity)
        g_reference = np.array([0.0, 0.0, 1.0])

        # Calculate rotation from measured to reference
        # This rotation transforms camera_init to world_init
        R = self.rotation_between_vectors(g_measured, g_reference)

        # Extract roll and pitch (yaw stays 0)
        roll, pitch, _ = euler_from_rotation_matrix(R)
        yaw = 0.0  # Force yaw to 0

        self.get_logger().info(
            f'Gravity alignment: roll={np.degrees(roll):.2f}°, '
            f'pitch={np.degrees(pitch):.2f}° (mean_acc: {mean_acc})'
        )

        # Convert to quaternion (negate for correct direction)
        q = quaternion_from_euler(-roll, -pitch, yaw)

        # Publish static TF: world_init → camera_init
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        # No translation (same origin)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Rotation: world_init (horizontal) → camera_init (tilted)
        q_inv = quaternion_inverse(q)
        t.transform.rotation.x = q_inv[0]
        t.transform.rotation.y = q_inv[1]
        t.transform.rotation.z = q_inv[2]
        t.transform.rotation.w = q_inv[3]

        self.tf_broadcaster.sendTransform(t)

        self.initialized = True
        self.get_logger().info(
            f'Published static TF: {self.parent_frame} → {self.child_frame}'
        )

        # Unsubscribe from IMU
        self.destroy_subscription(self.imu_sub)

    def rotation_between_vectors(self, v1, v2):
        """Calculate rotation matrix that aligns v1 to v2"""
        v1 = v1 / np.linalg.norm(v1)
        v2 = v2 / np.linalg.norm(v2)

        # Cross product and dot product
        cross = np.cross(v1, v2)
        dot = np.dot(v1, v2)

        # Handle parallel vectors
        if np.linalg.norm(cross) < 1e-6:
            if dot > 0:
                return np.eye(3)
            else:
                # 180 degree rotation around any perpendicular axis
                perp = np.array([1, 0, 0]) if abs(v1[0]) < 0.9 else np.array([0, 1, 0])
                axis = np.cross(v1, perp)
                axis = axis / np.linalg.norm(axis)
                # Rodrigues formula for 180 degree rotation: R = 2*a*a^T - I
                return 2 * np.outer(axis, axis) - np.eye(3)

        # Rodrigues formula
        s = np.linalg.norm(cross)
        c = dot

        vx = np.array([
            [0, -cross[2], cross[1]],
            [cross[2], 0, -cross[0]],
            [-cross[1], cross[0], 0]
        ])

        R = np.eye(3) + vx + vx @ vx * (1 - c) / (s * s)
        return R


def main(args=None):
    rclpy.init(args=args)
    node = WorldInitBroadcaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
