#!/usr/bin/env bash
# 하드웨어 없이 ROS 배선까지 확인하는 연기 테스트.
#
#   ./tools/ros-smoke.sh
#
# 하는 일:
#   1) 마커가 1.0m -> 0.085m 로 다가오는 합성 영상을 만든다
#   2) 그 접근 속도와 맞춘 가짜 /odom·/scan 을 발행한다
#   3) `drive` 를 영상 소스로 돌려서 DONE 까지 가는지, /cmd_vel 이 실제로 나가는지 본다
#   4) 센서를 안 띄운 채로도 한 번 돌려서 안전 게이트가 막는지 본다
#
# 단위 테스트(marker/tests)가 못 보는 것을 본다: rclpy 노드 생성, 토픽 배선,
# 카메라 어댑터, 종료 코드, 종료 시 정지 명령. 제어 성능은 여기서 판단하지 않는다
# (영상이 미리 녹화된 것이라 로봇 명령에 반응하지 않는다 — 그건 폐루프 시뮬레이션 몫).
set -eo pipefail
cd "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

[ -f /opt/ros/jazzy/setup.bash ] || { echo "[smoke] ROS2 jazzy 가 없습니다." >&2; exit 1; }
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"          # 실기 도메인과 섞이지 않게
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

WORK="$(mktemp -d)"
trap 'kill $(jobs -p) 2>/dev/null; rm -rf "$WORK"' EXIT
VIDEO="$WORK/approach.avi"

echo "[1/4] 합성 접근 영상 생성"
python3 - "$VIDEO" <<'PY'
import sys, cv2
sys.path.insert(0, ".")
from marker.tests.test_closed_loop_sim import Robot
out = cv2.VideoWriter(sys.argv[1], cv2.VideoWriter_fourcc(*"MJPG"), 12, (640, 480))
n, dist = 0, 1.0
for i in range(200):
    dist = 1.00 - i * 0.005
    if dist < 0.085:
        break
    frame = Robot(0.03 * dist, dist, 0.0).render()
    if frame is None:
        break
    out.write(frame)
    n += 1
out.release()
print(f"      {n} 프레임 (1.00m -> {dist:.3f}m)")
PY

echo "[2/4] 센서 없이 drive — 안전 게이트가 막아야 한다"
set +e
python3 -m marker.drive drive --source "$VIDEO" --slot front --rotate 0 --sensor-wait 2 \
    > "$WORK/gate.log" 2>&1
GATE_RC=$?
set -e
grep -q "센서 준비 안 됨" "$WORK/gate.log" && [ "$GATE_RC" = "3" ] \
    && echo "      OK: 종료코드 3, 누락 토픽을 지목함" \
    || { echo "      FAIL: 안전 게이트가 안 막았다(rc=$GATE_RC)"; tail -3 "$WORK/gate.log"; exit 1; }

echo "[3/4] 가짜 /odom·/scan 기동"
python3 - <<'PY' &
import math, time, rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
rclpy.init()
node = rclpy.create_node("smoke_fake_sensors")
po = node.create_publisher(Odometry, "/odom", 10)
ps = node.create_publisher(LaserScan, "/scan", 10)
t0, V = time.monotonic(), 0.06          # 영상의 접근 속도와 맞춘다
def tick():
    el = time.monotonic() - t0
    o = Odometry(); o.pose.pose.position.x = V * el; o.pose.pose.orientation.w = 1.0
    po.publish(o)
    s = LaserScan()
    s.angle_min, s.angle_max, s.angle_increment = -math.pi, math.pi, math.pi / 180
    s.range_min, s.range_max = 0.05, 12.0
    s.ranges = [max(0.9 - V * el, 0.09)] * 361
    ps.publish(s)
node.create_timer(1 / 20, tick)
try:
    rclpy.spin(node)
except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
    pass     # 스크립트가 끝내면서 죽이는 것이라 정상 종료다
PY
sleep 2

echo "[4/4] drive 실행 + /cmd_vel 감시"
python3 - > "$WORK/sink.log" 2>&1 <<'PY' &
import time, rclpy
from geometry_msgs.msg import Twist
rclpy.init()
node = rclpy.create_node("smoke_cmd_sink")
got = []
node.create_subscription(Twist, "/cmd_vel", lambda m: got.append((m.linear.x, m.angular.z)), 10)
t0 = time.monotonic()
while rclpy.ok() and time.monotonic() - t0 < 45:
    rclpy.spin_once(node, timeout_sec=0.1)
moving = [g for g in got if abs(g[0]) > 1e-9 or abs(g[1]) > 1e-9]
print(f"cmd_vel={len(got)} moving={len(moving)} last={got[-1] if got else None}")
PY
SINK=$!
sleep 1
set +e
timeout 60 python3 -m marker.drive drive --source "$VIDEO" --slot front --rotate 0 \
    --sensor-wait 5 --loop-hz 12 > "$WORK/drive.log" 2>&1
DRIVE_RC=$?
set -e
wait $SINK 2>/dev/null || true

echo "      drive 종료코드=$DRIVE_RC"
tail -2 "$WORK/drive.log" | sed 's/^/      /'
sed 's/^/      /' "$WORK/sink.log"

[ "$DRIVE_RC" = "0" ] || { echo "FAIL: drive 종료코드가 0 이 아니다($DRIVE_RC)"; exit 1; }
grep -q "\[ok\] 도착" "$WORK/drive.log" || { echo "FAIL: DONE 까지 못 감"; exit 1; }
grep -q "moving=[1-9]" "$WORK/sink.log" || { echo "FAIL: /cmd_vel 이 안 나갔다"; exit 1; }
grep -q "last=(0.0, 0.0)" "$WORK/sink.log" || { echo "FAIL: 종료 시 정지 명령이 없다"; exit 1; }
echo "[ok] ROS 배선 연기 테스트 통과"
