#!/usr/bin/env bash
# 로봇에서 실행 — ArUco 마커 한 장을 보고 마커 앞 10cm 까지 주행한다(nav2 미사용).
#
#   ./pi/drive.sh                       마커 ID 1 을 찾아 앞 10cm 까지
#   ./pi/drive.sh detect                모터 정지 상태로 검출값만(튜닝용)
#   ./pi/drive.sh detect --scan-dicts   벽 마커가 어떤 사전인지 확인
#   ./pi/drive.sh stop                  즉시 정지 — 모터를 물고 있으므로 비상구다
#   ./pi/drive.sh drive --steer-sign -1 --axis-gate 0.5     플래그는 그대로 위임
#
# ⚠️ 전제 1: 로봇 구동 노드가 떠 있어야 한다(/cmd_vel 구독, /odom·/scan 발행).
# ⚠️ 전제 2: 다른 스택(추종·영상송출·nav2)은 떠 있으면 안 된다.
#            카메라 장치와 /cmd_vel 을 다툰다.
# ⚠️ drive 모드는 /odom·/scan 이 들어올 때까지 최대 SENSOR_WAIT(기본 5)초 기다린다.
#    그 안에 안 들어오면 모터를 걸지 않고 종료코드 3 으로 빠진다.
# ⚠️ 시스템 python3 로 돈다 — picamera2 는 시스템 패키지라 venv 에서 안 잡힌다.
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

MODE="${1:-drive}"
case "$MODE" in
  drive|detect|stop) shift || true ;;
  -*) MODE="drive" ;;
  *) echo "[marker-drive] 모드는 drive|detect|stop 중 하나여야 합니다: $MODE" >&2; exit 1 ;;
esac

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "[marker-drive] ROS2 jazzy 가 없습니다(/opt/ros/jazzy/setup.bash)." >&2
  exit 1
fi
source /opt/ros/jazzy/setup.bash
python3 -c "import rclpy" 2>/dev/null || {
  echo "[marker-drive] rclpy 가 안 잡힙니다 — ROS2 설치·소싱을 확인하세요." >&2; exit 1; }

# robot-test.sh 가 현장에서 잡은 값. 있으면 읽고, 없으면 코드 기본값으로 간다.
# 이미 export 된 환경변수가 이기도록 field.env 쪽은 기본값 대입(:=)으로 넣는다.
if [ -f "$HERE/config/field.env" ]; then
  while IFS='=' read -r k v; do
    # 키가 셸 변수명 꼴이 아니면 건너뛴다. eval 에 그대로 넘기면 깨진 줄 하나가
    # 'bad substitution' 으로 주행 전체를 막는다(로그 문장이 값에 섞여 실제로 그랬다).
    case "$k" in
      [A-Za-z_]*) [[ "$k" =~ ^[A-Za-z_][A-Za-z0-9_]*$ ]] || { echo "[warn] field.env 의 이상한 줄 무시: $k" >&2; continue; } ;;
      *) continue ;;
    esac
    eval ": \${$k:=\$v}"
  done < "$HERE/config/field.env"
  # 도메인은 환경변수라야 rclpy 가 본다. 이게 틀리면 토픽이 안 보여서
  # '구동 노드가 없다'로 오진한다 — 실제로 그렇게 한 번 헤맸다.
  [ -n "${ROS_DOMAIN_ID:-}" ] && export ROS_DOMAIN_ID
fi

ARGS=("$MODE")
[ -n "${SOURCE:-}" ]           && ARGS+=(--source "$SOURCE")
[ -n "${SLOT:-}" ]             && ARGS+=(--slot "$SLOT")
[ -n "${ROTATE:-}" ]           && ARGS+=(--rotate "$ROTATE")
[ -n "${MARKER_ID:-}" ]        && ARGS+=(--marker-id "$MARKER_ID")
[ -n "${MARKER_M:-}" ]         && ARGS+=(--marker-m "$MARKER_M")
[ -n "${DICT:-}" ]             && ARGS+=(--dict "$DICT")
[ -n "${STOP_M:-}" ]           && ARGS+=(--stop-m "$STOP_M")
[ -n "${FRONT_OFFSET:-}" ]     && ARGS+=(--front-offset "$FRONT_OFFSET")
[ -n "${STEER_SIGN:-}" ]       && ARGS+=(--steer-sign "$STEER_SIGN")
[ -n "${AXIS_GATE:-}" ]        && ARGS+=(--axis-gate "$AXIS_GATE")
[ -n "${YAW_OFFSET_DEG:-}" ]   && ARGS+=(--yaw-offset-deg "$YAW_OFFSET_DEG")
[ -n "${STEER_ANG_MAX:-}" ]    && ARGS+=(--steer-ang-max "$STEER_ANG_MAX")
[ -n "${SCAN_FORWARD_DEG:-}" ] && ARGS+=(--scan-forward-deg "$SCAN_FORWARD_DEG")
[ -n "${SENSOR_WAIT:-}" ]      && ARGS+=(--sensor-wait "$SENSOR_WAIT")
[ -n "${CMD_TOPIC:-}" ]        && ARGS+=(--cmd-topic "$CMD_TOPIC")
[ -n "${LIN_PULSE:-}" ]        && ARGS+=(--lin-pulse "$LIN_PULSE")
[ -n "${LIN_HOMING:-}" ]       && ARGS+=(--lin-homing "$LIN_HOMING")
[ -n "${ANG_SEARCH:-}" ]       && ARGS+=(--ang-search "$ANG_SEARCH")
[ -n "${MOVE_PULSE_S:-}" ]     && ARGS+=(--move-pulse-s "$MOVE_PULSE_S")
[ -n "${MOVE_PAUSE_S:-}" ]     && ARGS+=(--move-pause-s "$MOVE_PAUSE_S")
[ -n "${TIMEOUT:-}" ]          && ARGS+=(--timeout "$TIMEOUT")
[ -n "${SCAN_GUARD:-}" ]       && ARGS+=(--scan-guard "$SCAN_GUARD")
[ -n "${POSE_YAW_TOL:-}" ]     && ARGS+=(--pose-yaw-tol "$POSE_YAW_TOL")
ARGS+=("$@")     # 명령줄 플래그가 맨 뒤 — argparse 는 뒤에 온 것이 이긴다

cd "$HERE"
exec python3 -m marker.drive "${ARGS[@]}"
