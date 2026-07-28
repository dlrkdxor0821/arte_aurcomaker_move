#!/usr/bin/env bash
# ┌──────────────────────────────────────────────────┐
# │  실행 위치: 노트북 (로봇 아님)                     │
# │  하드웨어 무접촉 — 카메라도 모터도 안 건드린다      │
# └──────────────────────────────────────────────────┘
# 로봇 없이 확인할 수 있는 것 전부.
#
#   ./laptop/check.sh            셋 다 (테스트 → ROS 배선 → 폐루프 시뮬 영상)
#   ./laptop/check.sh sim        폐루프 시뮬 영상만
#   ./laptop/check.sh sim --show 시뮬을 창으로 보면서 돌린다
#   ./laptop/check.sh sim --lat 0.25 --dist 1.1 --steer-sign 1
#
# 볼 수 없는 것: 실기 극성(--steer-sign), 카메라 장착각(--yaw-offset-deg),
# 실물 사전(--dict), 앞면 오프셋(--front-offset), 제동거리. 전부 로봇에서만 정해진다.
# 그건 로봇에서 ./robot-test.sh 로 잡는다.
set -eo pipefail
cd "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="out"; mkdir -p "$OUT"
printf '\033[36m[노트북 전용]\033[0m 하드웨어를 건드리지 않는다. 실기 값은 로봇에서 ./pi/setup.sh\n'

# 로봇에서 돌리면 안 된다. 이 스크립트는 가짜 /odom·/scan 을 뿌리고 /cmd_vel 을 발행한다
# (tools/ros-smoke.sh). 도메인을 77 로 격리하지만, 로봇이 마침 그 도메인이면 실제로 움직인다.
if [ "${1:-}" != "--force" ] && python3 -c "import picamera2" 2>/dev/null; then
  echo "[중단] picamera2 가 있다 = 여기는 로봇이다." >&2
  echo "       이 스크립트는 가짜 센서와 /cmd_vel 을 발행한다. 로봇에서 돌리지 마라." >&2
  echo "       로봇에서 할 것: ./pi/setup.sh" >&2
  echo "       그래도 돌리려면: ./laptop/check.sh --force  (모터 배선 분리 후)" >&2
  exit 1
fi
[ "${1:-}" = "--force" ] && shift

MODE="${1:-all}"
case "$MODE" in all|sim) shift || true ;; -*) MODE="all" ;;
  *) echo "모드는 all|sim: $MODE" >&2; exit 1 ;; esac

run_sim() {
  python3 - "$OUT/sim-approach.avi" "$@" <<'PY'
"""폐루프 시뮬레이션을 영상으로 남긴다 — 렌더→검출→상태기계→이동을 그대로 돌린다.

marker/tests/test_closed_loop_sim.py 의 Robot·카메라 약속을 그대로 재사용한다.
테스트는 통과/실패만 말하지만, 여기서는 로봇이 어떤 궤적으로 붙는지 눈으로 본다.
"""
import argparse, math, sys
import cv2
sys.path.insert(0, ".")
from marker.tests.test_closed_loop_sim import Robot, K, DIST, LEN_M, DICT, W, H
from marker.approach import MarkerApproach
from marker.config import MarkerDriveConfig
from marker.detect import detect_marker

ap = argparse.ArgumentParser()
ap.add_argument("path")
ap.add_argument("--lat", type=float, default=0.12, help="시작 좌우 이탈(m)")
ap.add_argument("--dist", type=float, default=0.9, help="시작 거리(m)")
ap.add_argument("--psi-deg", type=float, default=0.0, help="시작 정면각(도)")
ap.add_argument("--steer-sign", type=float, default=1.0)
ap.add_argument("--turn-sign", type=float, default=1.0, help="-1 = 배선 반대인 하드웨어")
ap.add_argument("--steer-ang-max", type=float, default=None, help="조향 각속도 상한(rad/s)")
ap.add_argument("--max-ticks", type=int, default=1200)
ap.add_argument("--show", action="store_true")
a = ap.parse_args()

kw = dict(steer_sign=a.steer_sign, stop_m=0.10, front_offset_m=0.0,
          timeout_s=1e9, align_stall_s=1e9, search_step_timeout_s=1e9,
          move_pause_s=0.15, lin_homing=0.10, lin_pulse=0.05)
if a.steer_ang_max is not None:
    kw["steer_ang_max"] = a.steer_ang_max
cfg = MarkerDriveConfig(**kw)
m, r = MarkerApproach(cfg), Robot(a.lat, a.dist, math.radians(a.psi_deg))
dt, t = 1.0 / cfg.loop_hz, 0.0
vw = cv2.VideoWriter(a.path, cv2.VideoWriter_fourcc(*"MJPG"), cfg.loop_hz, (W, H))
blank = None

for i in range(a.max_ticks):
    frame = r.render()
    obs = (detect_marker(frame, K, DIST, marker_len_m=LEN_M, target_id=1, dict_name=DICT)
           if frame is not None else None)
    cmd = m.step(obs, yaw_deg=r.odom_yaw_deg, forward_m=r.forward_m,
                 front_m=max(r.dist, 0.0), now_s=t)
    if frame is None:                       # 마커가 시야 밖 — 검은 화면으로 남긴다
        if blank is None:
            import numpy as np
            blank = np.zeros((H, W, 3), dtype="uint8")
        frame = blank.copy()
    for j, line in enumerate((
            f"{cmd.phase:<10} lin={cmd.linear:+.3f} ang={cmd.angular:+.3f}",
            f"dist={r.dist:+.3f}m  lat={r.lat:+.3f}m  psi={math.degrees(r.psi):+.1f}deg",
            f"tick={i}  marker={'yes' if obs else 'NO'}")):
        cv2.putText(frame, line, (8, 24 + 22 * j), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0) if obs is not None else (0, 0, 255), 2)
    vw.write(frame)
    if a.show:
        cv2.imshow("closed-loop sim", frame)
        if cv2.waitKey(int(1000 * dt)) & 0xFF == ord("q"):
            break
    if cmd.done:
        for _ in range(int(cfg.loop_hz)):   # 마지막 화면을 1초 물린다
            vw.write(frame)
        break
    r.move(cmd.linear, cmd.angular, dt, a.turn_sign)
    t += dt

vw.release()
if a.show:
    cv2.destroyAllWindows()
print(f"      결과: {cmd.phase}/{cmd.reason}  {i}틱  "
      f"dist={r.dist:.3f}m lat={r.lat:+.3f}m psi={math.degrees(r.psi):+.1f}deg")
print(f"      영상: {a.path}")
sys.exit(0 if cmd.phase == "DONE" else 1)
PY
}

if [ "$MODE" = "sim" ]; then
  echo "[sim] 폐루프 시뮬레이션"
  run_sim "$@" || echo "      (DONE 이 아님 — 위 이유를 본다)"
  exit 0
fi

echo "[1/3] 단위 테스트"
python3 -m pytest marker/tests/ -q 2>&1 | tail -1 | sed 's/^/      /'

echo "[2/3] ROS 배선 연기 테스트"
if [ -f /opt/ros/jazzy/setup.bash ]; then
  ./tools/ros-smoke.sh 2>&1 | tail -1 | sed 's/^/      /'
else
  echo "      건너뜀 — ROS2 jazzy 없음"
fi

echo "[3/3] 폐루프 시뮬레이션 (좌 12cm 이탈, 0.9m 에서 출발)"
run_sim || true
echo
echo "재생: xdg-open $OUT/sim-approach.avi"
echo "창으로 보기: ./laptop/check.sh sim --show"
