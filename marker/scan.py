"""원본 /scan 전방 섹터의 최소거리 — 유일한 근접 보호다.

필터된 스캔(scan_filtered)을 쓰지 않는 이유: 그쪽은 min_range 0.05 로 5cm 미만을
제거한다. 근접 감시에는 그 구간이 필요하다.
"""
import math

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanWatch:
    """.front_m 전방 최소거리(관측 전 None) · .ready 첫 /scan 수신 여부.

    .ready 가 필요한 이유: front_m 은 관측 전엔 None 인데, 상태기계는 front_m=None 을
    '근접 정보 없음(안전하다고 가정)'으로 취급한다. 그래서 /scan 이 아직 안 들어온
    상태를 front_m 만으로는 구분할 수 없고, 그 상태에서 움직이면 근접 안전장치가
    꺼진 채로 첫 몇 사이클을 도는 셈이 된다.
    """

    def __init__(self, node, topic: str = "/scan", half_angle_deg: float = 15.0):
        self.front_m = None
        self.ready = False
        self._half = math.radians(half_angle_deg)
        node.create_subscription(LaserScan, topic, self._on_scan, qos_profile_sensor_data)

    def _on_scan(self, msg) -> None:
        best = None
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue
            a = msg.angle_min + i * msg.angle_increment
            a = math.atan2(math.sin(a), math.cos(a))
            if abs(a) <= self._half and (best is None or r < best):
                best = r
        self.front_m = best
        self.ready = True
