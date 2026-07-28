#!/usr/bin/env bash
# 노트북(또는 로봇)에서 실행 — 마커 검출 오버레이. 모터를 건드리지 않는다.
#
#   ./pi/watch.sh                    노트북 USB 캠(index 0)으로 관찰
#   ./pi/watch.sh --scan-dicts       벽 마커가 어떤 사전인지 확인
#   ./pi/watch.sh --source csi --slot front --rotate 180    로봇 CSI 캠으로
#   ./pi/watch.sh --no-window        화면 없는 환경
#
# 용도: 마커가 몇 m 부터 잡히는지, yaw 가 어느 거리부터 안정되는지 보고
#       주행 쪽 --axis-gate 값을 정한다. ROS 가 필요 없다.
set -eo pipefail
cd "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 -m marker.watch "$@"
