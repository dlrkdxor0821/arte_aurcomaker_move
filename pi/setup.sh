#!/usr/bin/env bash
# ┌──────────────────────────────────────────────────┐
# │  실행 위치: 로봇(Pi) — 노트북에서 돌리면 0단계에서 │
# │  /odom·/scan·카메라가 없다고 막힌다               │
# │  5·6단계는 모터가 실제로 돈다                     │
# └──────────────────────────────────────────────────┘
# 실기에서만 정해지는 값을 하나씩 잡아 config/field.env 에 저장한다.
#
#   ./pi/setup.sh          0단계부터 끝까지
#   ./pi/setup.sh 3        3단계부터 (앞 단계 값은 이미 저장돼 있으므로 이어서)
#   ./pi/setup.sh reset    저장된 값 전부 지우고 처음부터
#
# 저장된 값은 pi/drive.sh 가 자동으로 읽는다. 그래서 이 스크립트를 한 번 끝내면
# 그 뒤로는 그냥 `./pi/drive.sh` 만 치면 된다.
#
# ⚠️ 5·6단계는 모터가 실제로 돈다. 시작 전 두 번째 터미널에 아래를 쳐두고 엔터만 남겨라:
#      ./pi/drive.sh stop
# set -e 는 안 쓴다. 이 스크립트는 "값이 없다/못 찾았다"를 정상 흐름으로 다루는데,
# grep 이 아무것도 못 찾아도 exit 1 이라 set -e 면 그 자리에서 조용히 죽는다
# (실제로 field.env 가 없을 때 0단계가 아무 말 없이 끝났다). 각 단계가 스스로
# 결과를 확인하고 fail=1 / exit 로 처리한다.
set -o pipefail
cd "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENVFILE="config/field.env"

# ROS 는 여기서 한 번만 소싱한다. 0단계 안에서 하면 `./pi/setup.sh 3` 처럼
# 건너뛰고 들어올 때 ros2 명령이 없는 채로 돈다.
# shellcheck disable=SC1091
[ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash || true

# ---------------------------------------------------------------- 도우미
say()  { printf '\n\033[1m%s\033[0m\n' "$*"; }
info() { printf '      %s\n' "$*"; }
ok()   { printf '      \033[32mOK\033[0m   %s\n' "$*"; }
bad()  { printf '      \033[31mFAIL\033[0m %s\n' "$*"; }
pause(){ read -r -p "      엔터로 계속 (Ctrl-C 중단) "; }

save() {   # save KEY VALUE — field.env 에 덮어쓴다
  mkdir -p config; touch "$ENVFILE"
  grep -v "^$1=" "$ENVFILE" > "$ENVFILE.tmp" || true
  printf '%s=%s\n' "$1" "$2" >> "$ENVFILE.tmp"
  mv "$ENVFILE.tmp" "$ENVFILE"
  ok "저장 $1=$2   ($ENVFILE)"
}

ask() {    # ask KEY "설명" 기본값 — 사용자 입력을 받아 저장. 빈 입력이면 기본값
  local v; read -r -p "      $2 [$3]: " v
  save "$1" "${v:-$3}"
}

# detect 를 N초만 돌려 출력을 그대로 돌려준다(모터 무접촉).
detect_run() { local s=$1; shift; timeout "$s" ./pi/drive.sh detect "$@" 2>&1 || true; }

# "z=0.812m" 같은 필드들의 중앙값. 흔들리는 한 프레임에 값이 끌려가지 않게.
median_of() { grep -oE "$1=[-+]?[0-9.]+" | cut -d= -f2 | sort -g | awk '{a[NR]=$1} END{if(NR)printf "%.3f", a[int((NR+1)/2)]}'; }

# ---------------------------------------------------------------- 단계
step0() {
  say "[0/6] 사전 점검 — 로봇이 준비됐는지"
  local fail=0
  [ -f /opt/ros/jazzy/setup.bash ] && ok "ROS2 jazzy (소싱됨)" || { bad "ROS2 jazzy 없음"; fail=1; }
  python3 -c "import rclpy" 2>/dev/null && ok "rclpy" || { bad "rclpy 안 잡힘"; fail=1; }
  python3 -c "import cv2" 2>/dev/null && ok "opencv" || { bad "opencv 없음"; fail=1; }
  python3 -c "import picamera2" 2>/dev/null && ok "picamera2 (CSI 카메라)" \
    || bad "picamera2 없음 — CSI 쓰려면: sudo apt install -y python3-picamera2"

  local topics; topics="$(timeout 5 ros2 topic list 2>/dev/null || true)"
  for t in /odom /scan; do
    grep -qx "$t" <<<"$topics" && ok "$t 발행 중" || { bad "$t 없음 — 구동 노드 확인"; fail=1; }
  done
  # 명령 토픽 이름은 로봇마다 다르다. field.env 에 CMD_TOPIC 이 있으면 그걸 본다.
  local topic; topic="$(grep -oE '^CMD_TOPIC=.*' "$ENVFILE" 2>/dev/null | cut -d= -f2 || true)"
  topic="${topic:-/cmd_vel}"
  local subs; subs="$(timeout 5 ros2 topic info "$topic" 2>/dev/null | grep -oE 'Subscription count: [0-9]+' | grep -oE '[0-9]+$' || echo 0)"
  if [ "${subs:-0}" -gt 0 ]; then
    ok "$topic 구독자 ${subs}개"
  else
    bad "$topic 구독자 0 — 명령을 내도 안 움직인다"
    fail=1
    # 구동 노드는 대개 /odom 을 발행한다. 그 노드가 뭘 구독하는지 보면 진짜 이름이 나온다.
    local drv; drv="$(timeout 5 ros2 topic info -v /odom 2>/dev/null \
      | awk '/Publishers:/{p=1} p&&/Node name:/{print $3; exit}')"
    if [ -n "$drv" ]; then
      info "구동 노드로 보이는 것: $drv — 이 노드가 구독하는 토픽:"
      timeout 5 ros2 node info "/$drv" 2>/dev/null \
        | awk '/Subscribers:/{p=1;next} /Publishers:|Service|Action/{p=0} p&&NF' | sed 's/^/        /'
    fi
    info "cmd 붙은 토픽 전부:"
    timeout 5 ros2 topic list 2>/dev/null | grep -i cmd | sed 's/^/        /' || true
    info "이름이 다르면: ./pi/setup.sh topic /진짜/이름"
  fi

  echo
  if [ "$fail" -ne 0 ]; then
    info "위 FAIL 을 먼저 해결해라. 해결 후: ./pi/setup.sh 0"
    exit 1
  fi
  info "다른 스택(추종·영상송출·nav2)이 꺼져 있는지 직접 확인해라 — 카메라와 /cmd_vel 을 다툰다."
  pause
}

step1() {
  say "[1/6] 마커 사전 확정 — 틀리면 검출 0개로 조용히 실패한다"
  info "마커가 카메라에 보이게 두고 계속."
  pause
  local out top
  out="$(detect_run 8 --scan-dicts)"
  echo "$out" | grep -E '검출된 사전' | tail -5 | sed 's/^/      /'
  top="$(echo "$out" | grep -oE 'DICT_[0-9A-Za-z_]+' | sort | uniq -c | sort -rn | head -1 | awk '{print $2}')"
  if [ -z "$top" ]; then
    bad "아무 사전도 안 잡혔다. 조명·초점·마커 크기 확인 후 다시: ./pi/setup.sh 1"
    exit 1
  fi
  ok "가장 많이 잡힌 사전: $top"
  ask DICT "사전 이름" "$top"
}

step2() {
  say "[2/6] 카메라 장착각 (--yaw-offset-deg)"
  info "로봇을 벽과 **직각**으로 세워라. 마커를 정면으로 마주보게."
  info "카메라가 비뚤게 달렸으면 여기서 0 이 아닌 yaw 가 나온다."
  pause
  local out y
  out="$(detect_run 8 --yaw-offset-deg 0)"
  echo "$out" | grep '^marker:' | tail -3 | sed 's/^/      /'
  y="$(echo "$out" | grep '^marker: z=' | median_of yaw)"
  [ -z "$y" ] && { bad "마커가 안 잡혔다. 1단계 사전값·마커 위치 확인"; exit 1; }
  ok "yaw 중앙값 ${y}도"
  ask YAW_OFFSET_DEG "카메라 장착각(도)" "$y"
}

step3() {
  say "[3/6] 앞면 오프셋 (--front-offset)"
  info "카메라와 로봇 최전방 사이 거리다. 로봇을 벽 앞에 두고 계속."
  pause
  local out z m
  out="$(detect_run 8 --front-offset 0)"
  echo "$out" | grep '^marker:' | tail -3 | sed 's/^/      /'
  z="$(echo "$out" | grep '^marker: z=' | median_of z)"
  [ -z "$z" ] && { bad "마커가 안 잡혔다"; exit 1; }
  ok "카메라~마커 거리 ${z}m"
  read -r -p "      자로 잰 (로봇 최전방~벽) 거리를 m 로 입력 [예: 0.35]: " m
  if [ -z "$m" ]; then
    save FRONT_OFFSET 0.0
  else
    save FRONT_OFFSET "$(awk -v a="$z" -v b="$m" 'BEGIN{printf "%.3f", a-b}')"
  fi
}

step4() {
  say "[4/6] yaw 신뢰 거리 (--axis-gate)"
  info "평면 마커는 먼 거리에서 자세 해가 둘이라 yaw 가 뒤집힌다."
  info "지금부터 20초간 검출값이 흐른다. 로봇을 **천천히 뒤로 물리면서**"
  info "yaw 가 흔들리기/뒤집히기 시작하는 거리(z)를 눈으로 잡아라."
  pause
  detect_run 20 | grep '^marker:' | sed 's/^/      /'
  echo
  ask AXIS_GATE "yaw 가 안정적이던 거리(m)" "0.6"
}

step5() {
  say "[5/6] 조향 극성 (--steer-sign) — ⚠️ 여기서부터 모터가 돈다"
  info "마커를 로봇 기준 **오른쪽 20cm** 치우치게, 거리 1m 에 둬라."
  info "로봇 앞 2m 를 비워라. 두 번째 터미널에 './pi/drive.sh stop' 준비."
  read -r -p "      준비됐으면 yes 입력: " c
  [ "$c" = "yes" ] || { info "중단. 준비되면: ./pi/setup.sh 5"; exit 0; }
  ./pi/drive.sh || true
  echo
  local d
  read -r -p "      로봇이 마커 '쪽으로' 틀었나? (y/n): " d
  if [ "$d" = "n" ]; then
    save STEER_SIGN -1
    info "극성 뒤집었다. 같은 배치로 다시 확인: ./pi/setup.sh 5"
  else
    save STEER_SIGN 1
  fi
}

step6() {
  say "[6/6] 정상 주행 + 정지 거리 보정 (--stop-m)"
  info "마커를 정면 1m, 치우침 없이 둬라. 앞 2m 비우고."
  read -r -p "      준비됐으면 yes 입력: " c
  [ "$c" = "yes" ] || { info "중단. 준비되면: ./pi/setup.sh 6"; exit 0; }
  ./pi/drive.sh || true
  echo
  local m
  read -r -p "      자로 잰 (로봇 최전방~벽) 실제 거리 m (건너뛰려면 엔터): " m
  if [ -n "$m" ]; then
    # 실제로 M 에 섰는데 목표가 0.10 이면, 목표를 (0.10 - (M-0.10)) 로 당긴다
    save STOP_M "$(awk -v m="$m" 'BEGIN{printf "%.3f", 0.20-m}')"
    info "다시 돌려서 확인해라: ./pi/drive.sh"
  fi
}

# ---------------------------------------------------------------- 진입점
if [ "${1:-}" = "reset" ]; then rm -f "$ENVFILE"; info "$ENVFILE 삭제"; exit 0; fi
if [ "${1:-}" = "topic" ]; then
  [ -n "${2:-}" ] || { echo "사용법: ./pi/setup.sh topic /진짜/토픽이름" >&2; exit 1; }
  save CMD_TOPIC "$2"; info "다시 점검: ./pi/setup.sh 0"; exit 0
fi
FROM="${1:-0}"
for n in 0 1 2 3 4 5 6; do
  [ "$n" -ge "$FROM" ] && "step$n"
done

say "완료 — 잡힌 값"
cat "$ENVFILE" 2>/dev/null | sed 's/^/      /'
echo
info "이제부터는 그냥: ./pi/drive.sh   (위 값을 자동으로 읽는다)"
info "값 하나만 다시: ./pi/setup.sh <단계번호>"
