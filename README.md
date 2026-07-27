# arte_aurcomaker_move — ArUco 마커 주행

마커 한 장을 보고 **마커 앞 10cm(로봇 최전방 기준)** 에 서는 주행. nav2 를 쓰지 않는다.
다른 저장소를 임포트하지 않는 단독 실행 레포다.

## 빠른 사용

```bash
# 0) 벽 마커가 어떤 사전인지 먼저 확정한다 (틀리면 검출 0개로 조용히 실패한다)
./marker-watch.sh --scan-dicts

# 1) 모터 없이 검출만 — 마커 부착 높이·조명 잡기
./marker-drive.sh detect

# 2) 실제 주행
./marker-drive.sh

# 3) 비상 정지
./marker-drive.sh stop
```

## 실행 전 확인

1. 로봇 구동 노드가 떠 있을 것 — `/cmd_vel` 구독, `/odom`·`/scan` 발행
2. 다른 스택(추종·영상송출·nav2)이 **꺼져 있을 것** — 카메라 장치와 `/cmd_vel` 을 다툰다
3. 마커가 카메라 높이에서 정면으로 보일 것
4. 시스템 python3 로 실행할 것 — `picamera2` 는 시스템 패키지라 venv 에서 안 잡힌다

`drive` 모드는 제어 루프에 들어가기 전에 `/odom`·`/scan` 이 최소 한 번 들어올 때까지
최대 `--sensor-wait`(기본 5)초 기다린다. 그 안에 안 들어오면 모터를 걸지 않고
어느 토픽이 안 들어왔는지 알려 준 뒤 종료코드 3 으로 빠진다. `front_m=None` 을
'근접 위험 없음'으로 읽는 상태기계 특성상, `/scan` 이 아직 없는 채로 움직이기
시작하면 근접 안전장치가 꺼진 첫 몇 사이클을 그냥 지나치기 때문이다. `detect`
모드는 모터를 건드리지 않으므로 기다리지 않고 매 줄에 센서 상태만 같이 찍는다.

## 동작

| 단계 | 하는 일 | 다음으로 |
|---|---|---|
| `SEARCH` | 좌우 ±60° 를 20° 씩 끊어 돌며 멈춰서 본다. 회전량은 `/odom` 으로 닫는다 | 검출 → `HOMING` / 1스윕 실패 → 중단 |
| `HOMING` | 화면 중앙 오차 PID 로 조향하며 전진. **yaw 는 안 쓴다**(먼 거리에서 뒤집힌다) | `z ≤ 0.6m` → `AXIS_ALIGN` |
| `AXIS_ALIGN` | yaw·축이탈로 마커 법선축에 올라탄다. 펄스 이동 | `z ≤ 0.10m` → 완료 / 정렬+근접 → `BLIND_PUSH` |
| `BLIND_PUSH` | 마커가 시야를 벗어난 마지막 구간을, 진입 시점 `/odom` 좌표에서의 **직선 변위**로 간다 | 도달 → 완료 |

7cm 마커는 약 12.6cm 부터 화면에서 잘리기 시작한다. 그래서 마지막 구간만 눈을 감는다.
직선 변위를 쓰는 이유: 누적 주행거리는 제자리 진동이나 회전으로도 늘어나서,
앞으로 안 갔는데 도달로 잘못 읽힐 수 있다.

## 종료코드

| 코드 | 의미 |
|---|---|
| `0` | 도착(`DONE`) 또는 `stop` 명령 정상 발행 |
| `1` | 상태기계가 `ABORT` 로 중단(`not_found`/`timeout`/`scan_guard`/`blocked`/`align_stall`/`turn_stall`/`bad_odom`) |
| `2` | 카메라 프레임을 못 받음 — 장치 확인 |
| `3` | `drive` 모드에서 `--sensor-wait` 안에 `/odom`·`/scan` 이 안 들어옴 |
| `130` | Ctrl-C |

## 실기에서만 정해지는 값

| 플래그 | 기본 | 정하는 법 |
|---|---|---|
| `--steer-sign` | `+1` | 로봇이 반대로 흐르면 `-1`. 소프트웨어로 판단 불가 |
| `--axis-gate` | `0.6` | `marker-watch.sh` 로 yaw 가 안정되는 거리를 보고 |
| `--front-offset` | `0.0` | `detect` 의 "앞면까지 남음" 과 자로 잰 값 비교 |
| `--dict` | `DICT_5X5_100` | `--scan-dicts` 로 실물 확정 |
| `--sensor-wait` | `5.0` | 로봇 구동 노드가 느리게 뜨면 늘린다 |

env 로도 준다: `STEER_SIGN=-1 ./marker-drive.sh`

## 구조

| 파일 | 책임 |
|---|---|
| `marker/approach.py` | **판단 전부.** 순수 상태기계 — 카메라·모터·시계·ROS 를 모른다 |
| `marker/detect.py` | 프레임 → 관측값. IPPE_SQUARE 고정, cv2 4.6/4.7+ 양쪽 지원 |
| `marker/camera.py` | **프레임 획득 이음매.** 다른 스택 위로 올릴 때 여기만 바꾼다 |
| `marker/calib.py` | 슬롯(front/back) → 캘리브 파일 |
| `marker/odom.py` `scan.py` | 누적 yaw·경로길이·최신 위치, 원본 `/scan` 전방 최소거리, 각각 `.ready` |
| `marker/drive.py` `watch.py` | CLI 배선(센서 준비 대기 포함) / 관찰 |

## 테스트

```bash
python3 -m pytest marker/tests/ -v
```

하드웨어 없이 돈다. 상태기계·검출·설정만 검증하고, ROS 어댑터와 카메라는 `detect` 모드
실행으로 확인한다.

## 문서

- `docs/design-marker-drive.md` — 설계와 근거(왜 게이트가 필요한가 등)
- `docs/prd-marker-drive.md` — 요구사항
- `docs/plan-marker-drive.md` — 구현 계획
