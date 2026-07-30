#!/usr/bin/env bash
# ┌──────────────────────────────────────────────────┐
# │  실행 위치: 로봇(Pi) — 노트북에서 돌리면 0단계에서 │
# │  /odom·/scan·카메라가 없다고 막힌다               │
# │  5·6·7단계는 모터가 실제로 돈다                   │
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
# ⚠️ 5·6·7단계는 모터가 실제로 돈다. 시작 전 두 번째 터미널에 아래를 쳐두고 엔터만 남겨라:
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
# 사람이 읽는 줄은 전부 stderr 로 낸다. stdout 은 함수의 '반환값' 전용이다.
# 섞이면 $(...) 로 값을 받을 때 로그 문장까지 값에 딸려 들어간다 —
# 실제로 LIN_PULSE 에 "…49회 발행 (구독자 1)\n0.05" 가 저장돼서 drive.sh 가 깨졌다.
say()  { printf '\n\033[1m%s\033[0m\n' "$*" >&2; }
info() { printf '      %s\n' "$*" >&2; }
ok()   { printf '      \033[32mOK\033[0m   %s\n' "$*" >&2; }
bad()  { printf '      \033[31mFAIL\033[0m %s\n' "$*" >&2; }
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

# 카메라를 이미 누가 잡고 있으면 모든 단계가 '검출 0개'로 보인다. 원인은 마커가
# 아니라 경합인데, 화면에는 똑같이 '안 잡혔다'로 나와서 조명·초점을 붙잡고 헤맨다.
cam_dev() { [ "$(get SOURCE)" = csi ] && echo /dev/video0 || echo "/dev/video$(get SOURCE)"; }
cam_busy() {   # 잡고 있는 프로세스가 있으면 그걸 알리고 0 을 돌려준다
  local dev pids p; dev="$(cam_dev)"
  [ -e "$dev" ] || { bad "$dev 가 없다 — 카메라 연결 확인"; return 0; }
  pids="$(fuser "$dev" 2>/dev/null | tr -s ' ')"
  [ -n "${pids// /}" ] || return 1
  bad "$dev 를 이미 쓰는 프로세스가 있다 —$pids"
  for p in $pids; do
    info "PID $p: $(tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null | cut -c1-80)"
  done
  info "끄기: kill$pids   (대개 './pi/watch.sh' 관찰 창이다)"
  return 0
}

# detect 를 N초만 돌려 출력을 그대로 돌려준다(모터 무접촉).
detect_run() {
  local s=$1; shift
  local out; out="$(timeout "$s" ./pi/drive.sh detect "$@" 2>&1 || true)"
  # 카메라를 못 열면 검출 줄이 아예 없다. 여기서 짚지 않으면 각 단계가
  # '마커가 안 잡혔다'로 오진한다 — 실제로 그 메시지에 두 번 속았다.
  if grep -q '영상 소스를 열 수 없다\|카메라 프레임이 없다' <<<"$out"; then
    bad "카메라를 못 열었다. 마커 문제가 아니다."
    cam_busy || info "장치를 잡은 프로세스는 없다 — 인덱스(SOURCE=$(get SOURCE))나 연결을 봐라"
  fi
  echo "$out"
}

get() {    # get KEY — 저장된 값(없으면 빈 문자열)
  grep -oE "^$1=.*" "$ENVFILE" 2>/dev/null | tail -1 | cut -d= -f2
}

# 진행 방향 쪽 끝단 이름. 뒷캠은 후진이라 자로 재는 면이 반대다 —
# '최전방~벽' 을 물어 놓고 뒤로 가면 오프셋이 로봇 길이만큼 통째로 틀린다.
edge() { [ "$(get SLOT)" = back ] && echo "최후방" || echo "최전방"; }
ahead() { [ "$(get SLOT)" = back ] && echo "뒤" || echo "앞"; }

# "z=0.812m" 같은 필드들의 중앙값. 흔들리는 한 프레임에 값이 끌려가지 않게.
median_of() { grep -oE "$1=[-+]?[0-9.]+" | cut -d= -f2 | sort -g | awk '{a[NR]=$1} END{if(NR)printf "%.3f", a[int((NR+1)/2)]}'; }

# ---------------------------------------------------------------- 단계
step0() {
  say "[0/7] 사전 점검 — 로봇이 준비됐는지"
  local fail=0
  [ -f /opt/ros/jazzy/setup.bash ] && ok "ROS2 jazzy (소싱됨)" || { bad "ROS2 jazzy 없음"; fail=1; }
  # 도메인이 다르면 토픽이 아예 안 보이거나 절반만 보인다. 새 터미널에서 또 틀리지
  # 않도록 지금 값을 저장해 두고, drive.sh 가 그대로 쓴다.
  ok "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
  save ROS_DOMAIN_ID "${ROS_DOMAIN_ID:-0}" >/dev/null

  # 어느 캠으로 갈지가 나머지 단계 전부의 전제다. 이걸 안 물으면 reset 뒤에 앞캠
  # 기본값으로 조용히 돌아가서, 뒷캠으로 잡은 장착각·오프셋을 앞캠 값으로 덮어쓴다.
  local cam c; cam="$(get SLOT)"
  read -r -p "      카메라 — front(CSI 앞캠, 전진) / back(USB 뒷캠, 후진) [${cam:-back}]: " c
  case "${c:-${cam:-back}}" in
    front) save SLOT front; save SOURCE csi; save ROTATE 180 ;;
    back)  save SLOT back;  save SOURCE 1;   save ROTATE 0 ;;
    *) bad "front 또는 back 만 된다"; exit 1 ;;
  esac
  [ "$(get SLOT)" = back ] && info "뒷캠 = **후진** 주행이다. 로봇 뒤를 비워라."
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
    # 우리 말고 다른 발행자가 있으면 명령이 덮인다. 대개 정지(0,0)를 주기적으로 쏘는
    # 브리지라서, 우리 명령이 나가자마자 0 으로 지워지고 로봇은 안 움직인다.
    # 에러가 안 나고 그냥 가만히 있어서 배선 문제로 오진하기 쉽다.
    local pubs; pubs="$(timeout 5 ros2 topic info -v "$topic" 2>/dev/null \
      | awk '/Node name:/{n=$3} /Endpoint type: PUBLISHER/{print n}' \
      | grep -v '^marker_drive$' | paste -sd, - || true)"
    if [ -n "$pubs" ]; then
      bad "$topic 에 다른 발행자가 있다: $pubs"
      info "이게 정지 명령을 계속 쏘면 우리 명령이 덮여서 로봇이 안 움직인다."
      info "확인: ros2 topic echo $topic   (가만히 둬도 값이 흐르면 그놈이다)"
      info "끄기: pgrep -af 'uvicorn|fastapi|aba_fms' 로 PID 찾아 kill"
      fail=1
    fi
  else
    bad "$topic 구독자 0 — 명령을 내도 안 움직인다"
    fail=1
    # cmd 붙은 토픽마다 '누가 듣고 있는지'를 보여준다. 구독자가 있는 쪽이 진짜다.
    info "cmd 붙은 토픽과 구독자:"
    local t n
    for t in $(timeout 5 ros2 topic list 2>/dev/null | grep -i cmd); do
      n="$(timeout 5 ros2 topic info -v "$t" 2>/dev/null \
           | awk '/Node name:/{name=$3} /Endpoint type: SUBSCRIPTION/{print name}' | paste -sd, -)"
      printf '        %-16s 구독자: %s\n' "$t" "${n:-없음}"
    done
    # 구동 노드는 대개 /odom 을 발행한다. 그 노드가 뭘 구독하는지 보면 확실하다.
    local drv; drv="$(timeout 5 ros2 topic info -v /odom 2>/dev/null \
      | awk '/Node name:/{n=$3} /Endpoint type: PUBLISHER/{print n; exit}')"
    if [ -n "$drv" ]; then
      info "/odom 을 발행하는 노드(=구동 노드로 보임): $drv"
      timeout 5 ros2 node info "/$drv" 2>/dev/null \
        | awk '/Subscribers:/{p=1;next} /Publishers:|Service |Service Clients|Action/{p=0} p&&NF' \
        | sed 's/^/        /'
    fi
    info "구독자가 있는 이름으로: ./pi/setup.sh topic /진짜/이름"
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
  say "[1/7] 마커 사전 확정 — 틀리면 검출 0개로 조용히 실패한다"
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

  # ID 도 여기서 잡는다. setup 이 안 물으면 코드 기본값(1)이 남는데, 벽 마커가
  # 0 이면 이후 모든 단계가 '마커가 안 잡혔다'로 죽는다 — 사전은 맞는데 ID 만
  # 틀린 경우라서 화면상 증상이 '아무것도 안 보임'과 똑같다. 실제로 그렇게 헤맸다.
  local raw ids first
  raw="$(echo "$out" | grep -oE "${top}\[[^]]*\]" | head -1)"
  ids="${raw#"${top}["}"; ids="${ids%]}"
  first="${ids%%,*}"; first="${first// /}"
  info "그 사전에서 보인 ID: ${ids:-없음}"
  ask MARKER_ID "따라갈 마커 ID" "${first:-1}"
}

step2() {
  say "[2/7] 카메라 장착각 (--yaw-offset-deg)"
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
  say "[3/7] 끝단 오프셋 (--front-offset)"
  info "카메라와 로봇 $(edge) 사이 거리다. 로봇 $(edge)이 벽을 마주보게 두고 계속."
  pause
  local out z m
  out="$(detect_run 8 --front-offset 0)"
  echo "$out" | grep '^marker:' | tail -3 | sed 's/^/      /'
  z="$(echo "$out" | grep '^marker: z=' | median_of z)"
  [ -z "$z" ] && { bad "마커가 안 잡혔다"; exit 1; }
  ok "카메라~마커 거리 ${z}m"
  read -r -p "      자로 잰 (로봇 $(edge)~벽) 거리를 m 로 입력 [예: 0.35]: " m
  if [ -z "$m" ]; then
    save FRONT_OFFSET 0.0
  else
    save FRONT_OFFSET "$(awk -v a="$z" -v b="$m" 'BEGIN{printf "%.3f", a-b}')"
  fi
}

step4() {
  say "[4/7] yaw 신뢰 거리 (--axis-gate)"
  info "평면 마커는 먼 거리에서 자세 해가 둘이라 yaw 가 뒤집힌다."
  info "지금부터 20초간 검출값이 흐른다. 로봇(또는 마커)을 **천천히 멀어지게** 하면서"
  info "yaw 가 흔들리기/뒤집히기 시작하는 거리(z)를 눈으로 잡아라."
  info "공간이 좁아 멀리 못 가면 그냥 엔터 — 기본값 0.6 이 7cm 마커에 맞는 값이다."
  pause
  detect_run 20 | grep '^marker:' | sed 's/^/      /'
  echo
  ask AXIS_GATE "yaw 가 안정적이던 거리(m)" "0.6"
}

cmd_topic() {   # 저장된 명령 토픽(없으면 /cmd_vel)
  local t; t="$(grep -oE '^CMD_TOPIC=.*' "$ENVFILE" 2>/dev/null | cut -d= -f2 || true)"
  echo "${t:-/cmd_vel}"
}

nudge() {       # nudge LIN ANG SECS — 상태기계를 빼고 직접 명령을 쏜다
  # `ros2 topic pub` 을 timeout 으로 자르면 DDS 탐색(1초 가까이)이 끝나기 전에 죽어서
  # 구독자에게 한 건도 안 갈 수 있다. 노드를 직접 띄우고 구독자를 기다린 뒤 쏜다.
  python3 - "$(cmd_topic)" "$1" "$2" "$3" <<'PY'
import sys, time, rclpy
from geometry_msgs.msg import Twist
topic, lin, ang, secs = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
rclpy.init()
node = rclpy.create_node("marker_setup_nudge")
pub = node.create_publisher(Twist, topic, 10)
end = time.monotonic() + 2.0                       # 구독자가 붙을 때까지 기다린다
while time.monotonic() < end and pub.get_subscription_count() == 0:
    rclpy.spin_once(node, timeout_sec=0.05)
n = pub.get_subscription_count()
if n == 0:
    print(f"        [경고] {topic} 구독자가 0 이다 — 명령이 아무 데도 안 간다", file=sys.stderr)
msg = Twist(); msg.linear.x = lin; msg.angular.z = ang
end = time.monotonic() + secs
sent = 0
while time.monotonic() < end:
    pub.publish(msg); sent += 1
    rclpy.spin_once(node, timeout_sec=0.05)
pub.publish(Twist())                                # 정지
rclpy.spin_once(node, timeout_sec=0.1)
print(f"        {topic} 로 lin={lin} ang={ang} 를 {sent}회 발행 (구독자 {n})", file=sys.stderr)
node.destroy_node(); rclpy.shutdown()
PY
}

# 명령 토픽의 실체를 보여준다 — 타입·구독자. 안 움직일 때 이게 답을 준다.
show_cmd_topic() {
  local t; t="$(cmd_topic)"
  info "$t 상태:"
  timeout 5 ros2 topic info -v "$t" 2>&1 \
    | grep -E 'Type:|Node name:|Endpoint type:|count:' | sed 's/^/        /'
}

# 값을 키워 가며 "움직였나"를 물어, 처음 움직인 값을 돌려준다.
find_threshold() {   # find_threshold "설명" KIND 값...
  local label="$1" kind="$2"; shift 2
  local v a
  for v in "$@"; do
    info "$label $v — 2.5초 명령을 쏜다"
    if [ "$kind" = lin ]; then nudge "$v" 0 2.5; else nudge 0 "$v" 2.5; fi
    read -r -p "      움직였나? (y/n/q=중단): " a
    [ "$a" = q ] && return 1
    [ "$a" = y ] && { echo "$v"; return 0; }
  done
  return 1
}

step5() {
  say "[5/7] 모터 최소 명령값 — ⚠️ 여기서부터 모터가 돈다"
  info "기본값(전진 0.05m/s, 회전 0.08rad/s)은 옛 코드의 바퀴 퍼센트 단위에서 옮겨온 값이라"
  info "실제 m/s 로는 모터 불감대 아래일 수 있다. 실제로 도는 최소값을 찾는다."
  info "**로봇을 들어 올려 바퀴를 띄우는 것이 가장 안전하다** — 공간이 필요 없다."
  info "바닥에 둘 거면 앞뒤를 비워라. 바퀴가 도는지만 보면 된다."
  read -r -p "      준비됐으면 yes 입력: " c
  [ "$c" = "yes" ] || { info "중단. 준비되면: ./pi/setup.sh 5"; exit 0; }

  local lin ang
  lin="$(find_threshold "전진" lin 0.05 0.08 0.12 0.18 0.25)" || {
    bad "어느 값에서도 안 움직였다 — 이건 값 문제가 아니라 배선/토픽 문제다."
    info "확인: ros2 topic info -v $(cmd_topic)   (구독자 노드·메시지 타입)"
    info "타입이 TwistStamped 면 우리는 Twist 를 쏘고 있어 안 맞는다 — 알려달라."
    exit 1; }
  save LIN_PULSE "$lin"
  save LIN_HOMING "$(awk -v v="$lin" 'BEGIN{printf "%.3f", v*2}')"

  ang="$(find_threshold "회전" ang 0.08 0.15 0.25 0.40 0.60)" || {
    bad "회전이 안 된다 — 배선 확인"; exit 1; }
  save STEER_ANG_MAX "$ang"
  save ANG_SEARCH "$(awk -v v="$ang" 'BEGIN{printf "%.3f", v*2}')"

  # 펄스 길이와 전체 제한시간을 이 로봇 속도에 맞춘다.
  #
  # 펄스가 옮기는 거리 = move_pulse_s x lin_pulse 다. 시간이 고정돼 있으면 느린 로봇은
  # 한 펄스에 몇 mm 밖에 못 가고, 펄스 사이 정지(0.9초)까지 더해져 실효 속도가 폭락한다.
  # 실측: lin_pulse 0.05 면 펄스당 8.3mm, 실효 7.8mm/s → 50cm 구간에 64초.
  # 기본 제한시간 60초라 도착 직전에 timeout 으로 중단된다.
  # 그래서 시간이 아니라 **거리(2cm)** 를 기준으로 펄스 길이를 정한다.
  local pulse timeout eff
  pulse="$(awk -v v="$lin" 'BEGIN{p=0.02/v; if(p<0.167)p=0.167; if(p>1.0)p=1.0; printf "%.3f", p}')"
  save MOVE_PULSE_S "$pulse"
  eff="$(awk -v v="$lin" -v p="$pulse" 'BEGIN{printf "%.5f", v*p/(p+0.9)}')"
  # 0.6m(게이트~정지) 를 실효 속도로 가는 시간의 3배 + 여유 40초. 넉넉해도 손해는
  # 없다 — 진짜 막히면 blocked/align_stall 이 먼저 잡는다. timeout 은 마지막 그물이다.
  timeout="$(awk -v e="$eff" 'BEGIN{t=0.6/e*3+40; if(t<120)t=120; if(t>600)t=600; printf "%.0f", t}')"
  save TIMEOUT "$timeout"
  info "실효 전진 속도 $(awk -v e="$eff" 'BEGIN{printf "%.0f", e*1000}')mm/s → 펄스 ${pulse}초, 제한시간 ${timeout}초"
}

step6() {
  say "[6/7] 조향 극성 (--steer-sign) — ⚠️ 모터가 돈다"
  info "마커를 로봇 기준 **한쪽으로** 치우치게 둬라 — 왼쪽·오른쪽 아무 쪽이나."
  info "10cm 면 충분하다(많을수록 뚜렷). 판정은 '어느 쪽인가'가 아니라"
  info "'마커 쪽으로 트는가 반대로 트는가'라서 어느 쪽에 두든 같다."
  info "거리는 0.6~1.2m 아무 데나. 좁으면 짧게 해도 극성은 보인다."
  info "로봇 $(ahead)를 비워라(진행 방향). 두 번째 터미널에 './pi/drive.sh stop' 준비."
  info "볼 것은 '도착했나'가 아니라 **처음 몇 초에 어느 쪽으로 트는가** 하나뿐이다."
  read -r -p "      준비됐으면 yes 입력: " c
  [ "$c" = "yes" ] || { info "중단. 준비되면: ./pi/setup.sh 6"; exit 0; }
  ./pi/drive.sh; local rc=$?
  echo
  if [ "$rc" -ge 2 ]; then
    bad "주행이 시작도 못 했다(종료코드 $rc). 극성 판단은 의미가 없다 — 먼저 해결해라."
    exit 1
  fi
  # 종료코드 1(ABORT)은 여기서 정상이다. 좁은 공간이라 도착 못 해도 조향 방향은 봤다.
  local d
  read -r -p "      로봇이 마커 '쪽으로' 틀었나? (y/n): " d
  if [ "$d" = "n" ]; then
    save STEER_SIGN -1
    info "극성 뒤집었다. 같은 배치로 다시 확인: ./pi/setup.sh 6"
  else
    save STEER_SIGN 1
  fi
}

step7() {
  say "[7/7] 정상 주행 + 정지 거리 보정 (--stop-m)"
  info "마커를 정면에, 치우침 없이 둬라. 거리는 **낼 수 있는 최대**(0.8m 면 충분)."
  info "로봇 $(ahead)를 비워라(진행 방향). 두 번째 터미널에 './pi/drive.sh stop' 준비."
  read -r -p "      준비됐으면 yes 입력: " c
  [ "$c" = "yes" ] || { info "중단. 준비되면: ./pi/setup.sh 7"; exit 0; }
  ./pi/drive.sh; local rc=$?
  echo
  if [ "$rc" -ne 0 ]; then
    bad "도착하지 못했다(종료코드 $rc). 위 마지막 줄의 중단 이유를 보고 README 표를 확인해라."
    info "여기서 거리를 재서 보정하면 안 된다 — 안 움직인 거리를 목표로 삼게 된다."
    exit 1
  fi
  local m new want
  # 지금 목표를 읽어서 쓴다. 0.10 을 박아 두면 STOP_M 을 손으로 바꾼 뒤 이 단계가
  # 옛 목표 기준으로 보정해서, 맞춰 놓은 거리를 조용히 되돌린다.
  want="$(get STOP_M)"; want="${want:-0.10}"
  info "현재 목표 STOP_M=${want}m (로봇 $(edge)~벽)"
  read -r -p "      자로 잰 (로봇 $(edge)~벽) 실제 거리 m (건너뛰려면 엔터): " m
  [ -n "$m" ] || return 0
  # 목표 W 인데 실제 M 에 섰으면 오차만큼 목표를 당긴다: W - (M - W)
  new="$(awk -v m="$m" -v w="$want" 'BEGIN{printf "%.3f", 2*w-m}')"
  # 7cm 마커는 약 12.6cm 부터 화면에서 잘린다. 목표를 너무 당기면 눈감고 가는 구간만 길어진다.
  if awk -v v="$new" 'BEGIN{exit !(v<0.05 || v>0.30)}'; then
    bad "보정값 ${new}m 는 범위 밖이다(0.05~0.30). 측정이나 주행이 잘못됐을 가능성이 크다."
    info "저장하지 않는다. 주행을 다시 확인해라: ./pi/drive.sh"
    return 0
  fi
  save STOP_M "$new"
  info "다시 돌려서 확인해라: ./pi/drive.sh"
}

# ---------------------------------------------------------------- 진입점
if [ "${1:-}" = "reset" ]; then rm -f "$ENVFILE"; info "$ENVFILE 삭제"; exit 0; fi
if [ "${1:-}" = "topic" ]; then
  [ -n "${2:-}" ] || { echo "사용법: ./pi/setup.sh topic /진짜/토픽이름" >&2; exit 1; }
  save CMD_TOPIC "$2"; info "다시 점검: ./pi/setup.sh 0"; exit 0
fi
FROM="${1:-0}"
for n in 0 1 2 3 4 5 6 7; do
  [ "$n" -ge "$FROM" ] && "step$n"
done

say "완료 — 잡힌 값"
cat "$ENVFILE" 2>/dev/null | sed 's/^/      /'
echo
info "이제부터는 그냥: ./pi/drive.sh   (위 값을 자동으로 읽는다)"
info "값 하나만 다시: ./pi/setup.sh <단계번호>"
