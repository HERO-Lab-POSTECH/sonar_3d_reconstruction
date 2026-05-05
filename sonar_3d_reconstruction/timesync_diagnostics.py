"""Builds diagnostic_msgs/DiagnosticArray for timesync state (spec §2.10.2)."""
from collections import deque

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class RollingMean:
    def __init__(self, n: int = 50):
        self._buf: deque = deque(maxlen=n)

    def add(self, value: float) -> None:
        self._buf.append(float(value))

    def value(self) -> float:
        return sum(self._buf) / len(self._buf) if self._buf else 0.0


class TimesyncDiagnostics:
    def __init__(self):
        self.stamp_diff_mean = RollingMean(50)
        self.odom_age_mean = RollingMean(50)
        self.paired_count = 0
        self.dropped_stamp_diff = 0
        self.dropped_stale_odom = 0
        self.dropped_quality_gate = 0
        self.policy = 'latest'

    def to_msg(self, node_clock_now) -> DiagnosticArray:
        msg = DiagnosticArray()
        msg.header.stamp = node_clock_now.to_msg()
        status = DiagnosticStatus()
        status.name = 'sonar_3d_mapper: timesync'
        status.hardware_id = ''
        if self.dropped_stale_odom > 0 or self.dropped_stamp_diff > 0:
            status.level = DiagnosticStatus.WARN
            status.message = 'Some frames dropped'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'Healthy'
        status.values = [
            KeyValue(key='stamp_diff_mean_sec', value=f'{self.stamp_diff_mean.value():.4f}'),
            KeyValue(key='odom_age_mean_sec', value=f'{self.odom_age_mean.value():.4f}'),
            KeyValue(key='paired_count', value=str(self.paired_count)),
            KeyValue(key='dropped_stamp_diff', value=str(self.dropped_stamp_diff)),
            KeyValue(key='dropped_stale_odom', value=str(self.dropped_stale_odom)),
            KeyValue(key='dropped_quality_gate', value=str(self.dropped_quality_gate)),
            KeyValue(key='policy', value=self.policy),
        ]
        msg.status.append(status)
        return msg
