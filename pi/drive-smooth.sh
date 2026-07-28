#!/usr/bin/env bash
# ┌──────────────────────────────────────────────────┐
# │  실행 위치: 로봇(Pi) — 모터가 돈다                │
# │  실험용: 끊지 않고 연속으로 주행해 본다           │
# └──────────────────────────────────────────────────┘
#
#   ./pi/drive-smooth.sh            연속 주행 (정지 없음)
#   ./pi/drive-smooth.sh 0.3        0.3초씩만 쉬며 주행 (중간 단계)
#   ./pi/drive-smooth.sh 0 --steer-sign -1     남는 인자는 drive.sh 로 그대로
#
# 기본 주행(./pi/drive.sh)은 AXIS_ALIGN 구간에서 0.167초 가고 0.9초 멈춘다.
# 전체 틱의 17% 만 전진 명령이 나가는 셈이다. 멈춘 순간의 흐리지 않은 프레임으로만
# 마커 자세를 풀기 위해서다 — ArUco 는 코너 4개의 픽셀 위치로 거리·각도를 계산하는데,
# 움직이며 찍으면 모션블러로 그 코너가 뭉개진다.
#
# 이 스크립트는 그 정지를 없앤다(전진 명령 100%). 코드는 안 바꾼다 — 설정만 덮는다.
#
# 볼 것: 더 빠른 대신 정렬이 흔들리는가. 마지막에 소요 시간과 결과가 찍히므로
#        기본 주행과 숫자로 비교할 수 있다.
#
# ⚠️ 두 번째 터미널에 './pi/drive.sh stop' 을 쳐두고 엔터만 남겨라.
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

PAUSE="0"
case "${1:-}" in
  ''|-*) ;;                     # 인자 없음 또는 바로 플래그 → 기본 0
  *) PAUSE="$1"; shift ;;
esac

DUTY="$(awk -v p="$PAUSE" 'BEGIN{printf "%.0f", 100*0.167/(0.167+p)}')"
echo "[연속 주행 실험] 펄스 사이 정지 ${PAUSE}초 — 전진 명령이 전체의 약 ${DUTY}%"
[ "$PAUSE" = "0" ] && echo "                 (기본 주행은 0.9초 정지, 약 17%)"
echo "                 흐린 프레임으로 판단하게 되므로 정렬이 흔들릴 수 있다."
echo

START="$SECONDS"
set +e
MOVE_PAUSE_S="$PAUSE" "$HERE/pi/drive.sh" drive "$@"
RC=$?
set -e
ELAPSED=$((SECONDS - START))

echo
echo "[결과] 종료코드=$RC  소요 ${ELAPSED}초"
case "$RC" in
  0) echo "       도착. 기본 주행(./pi/drive.sh)과 시간을 비교해 봐라."
     echo "       정렬도 만족스러우면 계속 쓰려면 field.env 에 넣는다:"
     echo "         echo MOVE_PAUSE_S=$PAUSE >> config/field.env" ;;
  1) echo "       중단됐다. 위 마지막 줄의 이유를 봐라 —"
     echo "       align_stall / lost_misaligned 면 모션블러로 자세가 흔들린 것이다."
     echo "       중간값으로 다시: ./pi/drive-smooth.sh 0.3" ;;
  *) echo "       주행 전에 막혔다(센서·카메라). README 종료코드 표 참고." ;;
esac
