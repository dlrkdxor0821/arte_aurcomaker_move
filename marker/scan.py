"""원본 /scan 전방 섹터의 최소거리 — 유일한 근접 보호다.

필터된 스캔(scan_filtered)을 쓰지 않는 이유: 그쪽은 min_range 0.05 로 5cm 미만을
제거한다. 근접 감시에는 그 구간이 필요하다.
"""
import math
import time

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

    def age(self) -> float:
        """마지막 스캔이 들어온 뒤 흐른 시간(초). 한 번도 없으면 무한대.

        '한 번 받았다'와 '지금 살아 있다'는 다른 말이다. 라이다가 죽거나 DDS 가
        끊기면 마지막 거리값이 영원히 남아, 근접 보호가 켜진 것처럼 보인 채로 꺼진다.
        """
        return float("inf") if self._last_t is None else time.monotonic() - self._last_t

    def _on_scan(self, msg) -> None:
        best = None
        for i, r in enumerate(msg.ranges):
            # LaserScan 규약: range_min~range_max 밖의 값은 무효다. 그 밖의 유한값을
            # 그대로 믿으면 없는 장애물에 서게 된다.
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            a = msg.angle_min + i * msg.angle_increment
            a = math.atan2(math.sin(a), math.cos(a))
            if abs(a) <= self._half and (best is None or r < best):
                best = r
        self.front_m = best
        self._last_t = time.monotonic()
        self.ready = True
        self.ready = True
