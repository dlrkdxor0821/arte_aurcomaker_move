#!/usr/bin/env bash
# 노트북(또는 로봇)에서 실행 — 마커 검출 오버레이. 모터를 건드리지 않는다.
#
#   ./pi/watch.sh                    노트북 USB 캠(index 0)으로 관찰
#   ./pi/watch.sh --scan-dicts       벽 마커가 어떤 사전인지 확인
#   ./pi/watch.sh --source csi --slot front --rotate 180    로봇 CSI 캠으로
#   ./pi/watch.sh --no-window        화면 없는 환경
#
#   # 뒷캠이 마커를 잡는지 눈으로 — 로봇에서 띄우고 노트북 브라우저로 본다
#   ./pi/watch.sh --source 1 --slot back --rotate 0 --http 8088
#   →  http://<로봇IP>:8088
#
# 용도: 마커가 몇 m 부터 잡히는지, yaw 가 어느 거리부터 안정되는지 보고
#       주행 쪽 --axis-gate 값을 정한다. ROS 가 필요 없다.
# '검출 0개'는 원인이 다섯이다(사전·ID·크기·밝기·화면 밖). 그래서 잡힌 마커를 전부
# 그리고, 하나도 못 잡으면 1초에 한 번 모든 사전을 훑어 화면과 로그에 적는다.
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"

# drive.sh 와 **같은** 값을 본다. 안 그러면 관찰 화면은 id 1 을 찾고 주행은 id 0 을
# 찾는 상태가 되고, 그건 어디에도 표시가 안 난다(실제로 그렇게 헤맸다).
# shellcheck source=pi/field.sh
source "$HERE/pi/field.sh"

ARGS=()
[ -n "${SOURCE:-}" ]    && ARGS+=(--source "$SOURCE")
[ -n "${SLOT:-}" ]      && ARGS+=(--slot "$SLOT")
[ -n "${ROTATE:-}" ]    && ARGS+=(--rotate "$ROTATE")
[ -n "${MARKER_ID:-}" ] && ARGS+=(--marker-id "$MARKER_ID")
[ -n "${MARKER_M:-}" ]  && ARGS+=(--marker-m "$MARKER_M")
[ -n "${DICT:-}" ]      && ARGS+=(--dict "$DICT")
[ -n "${STOP_M:-}" ]    && ARGS+=(--stop-m "$STOP_M")
ARGS+=("$@")     # 명령줄이 맨 뒤 — argparse 는 뒤에 온 것이 이긴다

exec python3 -m marker.watch "${ARGS[@]}"
