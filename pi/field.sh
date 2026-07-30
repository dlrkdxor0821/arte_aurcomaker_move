# config/field.env 를 환경변수로 올린다. source 전용 — 실행 파일이 아니다.
#
# drive.sh 와 watch.sh 가 같은 값을 봐야 한다. 한쪽만 읽으면 관찰 화면과 실제 주행이
# 다른 마커·다른 캠을 보게 되고, 그건 화면에 아무 표시도 안 난다 — 실제로 watch 가
# id 1 을 찾는 동안 drive 는 id 0 을 찾고 있었다.
#
# 이미 export 된 환경변수가 이기도록 기본값 대입(:=)으로 넣는다.
# HERE 는 부르는 쪽이 정한다(레포 루트).
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
  if [ -n "${ROS_DOMAIN_ID:-}" ]; then export ROS_DOMAIN_ID; fi
fi
# source 의 반환값 = 마지막 명령의 반환값이다. 위 조건이 거짓으로 끝나면 set -e 를
# 쓰는 호출자가 여기서 조용히 죽는다(아무것도 안 찍고 종료코드 1).
:
