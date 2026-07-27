# 마커 주행(Marker Drive) 설계 — 2026-07-27

ArUco 마커 한 장을 보고 **마커 앞 10cm(로봇 최전방 기준)** 에 서는 주행. nav2 를 쓰지 않는다.

이 문서는 `aba_project` 에서 시작한 설계를 이 레포로 옮기면서 **grill-me 인터뷰에서 뒤집힌
결정을 전부 반영해 다시 쓴 것**이다. 이전 판(4cm 정지·초음파 판정·슬롯 1개 등)은 폐기됐다.

## 왜 별도 레포인가

1단계는 **튜닝 리그**다. `aba_project` 스택(추종·영상송출·nav2)을 내리고 이 레포만 로봇에서
돌리면 골칫거리 셋이 한꺼번에 사라진다:

| 문제 | aba 스택 위에서 | 단독 실행 |
|---|---|---|
| 카메라 장치 경합 | `camera_sender` 가 잡고 있어 직접 열면 `Device or resource busy` | 없음 — 직접 연다 |
| `/cmd_vel` 경합 | `cmd_bridge` 가 명령 없어도 20Hz 정지 Twist 발행 | 없음 — `follow-drive` 창이 안 뜬다 |
| nav2 목표 점유 | 주행목표 취소 결함 수정 대기 | 없음 — nav2 를 안 띄운다 |

대가는 3단계(BT 승격) 때 프레임 획득 경로가 바뀐다는 것 하나다. 그건 **이음매 하나**로 막는다.

## 대상 하드웨어 / 마커

| 항목 | 값 |
|---|---|
| 마커 사전 | `DICT_5X5_100` (기본값). 5X5 계열은 `_50/_100/_250` 이 비트 패턴이 달라 **틀리면 검출 0개로 조용히 실패** — `detect --scan-dicts` 로 확정한다 |
| 마커 ID | 1 |
| 마커 한 변 | 0.07 m |
| 부착 위치 | 벽 |
| 정지 거리 | **마커에서 로봇 최전방까지 0.10 m** |
| 카메라 | 앞캠 picam(`rotate 180`). 뒷캠 USB 는 3단계 |

## 물리적 제약 — 10cm 근처에서 마커가 시야를 넘는다

실제 캘리브 값 기준 계산(앞캠 `fx=609.2`, `fy=607.4`, 640×480):

```
7cm 마커가 화면 세로에서 차지하는 비율
   z = 8.9cm  → 100%   검출 불가
   z = 12.6cm →  70%   안정 검출 하한
   z = 15cm   →  57%   여유
```

즉 **약 12.6cm 까지는 확실히 보이고, 목표 10cm 까지 남는 2~3cm 는 시각 정보 없이 간다.**
그 구간은 `/odom` 누적 전진거리로 잰다. 3cm 구간의 odom 오차는 1mm 미만, 마커 거리
추정 오차(실측 3%)는 12.6cm 에서 4mm — 합쳐도 목표 대비 ±5mm 수준이다.

(참고: 이전 판의 4cm 목표는 물리적으로 불가능했다. 4cm 에서 화각은 가로 4.2cm·세로 3.2cm 라
7cm 마커가 프레임을 넘는다. 10cm 로 올리면서 중첩 마커 안이 필요 없어졌다.)

## 접근법 — 거리 게이트 2단

| 안 | 기각 사유 |
|---|---|
| 순수 이미지 서보(`ex` PID 만) | 축이탈 보정 불가 — 비스듬히 접근하면 비스듬히 도착 |
| 마커맵 + EKF | 마커 한 장에 과함 (YAGNI) |
| **거리 게이트 2단 (채택)** | — |

단일 평면 마커의 자세는 해가 둘이다(`IPPE_SQUARE` 도 마찬가지). 멀리서 yaw 가 뒤집히므로
**게이트(기본 0.6m) 밖에서는 yaw 를 쓰지 않고 `ex` 만** 쓴다. 근거리에서는 코너 각도차가
커져 해가 갈리므로 그때부터 yaw·축이탈을 쓴다. 마커를 늘리거나 ChArUco 로 가지 않아도 되는
가장 싼 대책이다.

## 구조

```
marker/
  types.py     MarkerObs, Cmd            값 전달만
  config.py    MarkerDriveConfig         튜닝 상수 기본값
  approach.py  MarkerApproach            ★ 판단 전부. 순수 상태기계
  detect.py    detect_marker             프레임 → MarkerObs (IPPE_SQUARE)
  calib.py     load_calib                슬롯 → (K, dist)
  camera.py    get_frame                 ★ 프레임 획득 이음매
  odom.py      OdomTracker               /odom → 누적 yaw, 누적 전진거리
  scan.py      ScanWatch                 원본 /scan 전방 최소거리
  drive.py     CLI 배선
  watch.py     노트북 관찰
```

`approach.py` 만 판단하고 나머지는 값을 나른다. 그래서 자동 테스트 대상도 그 파일 하나다.

### 이음매 — `camera.get_frame()`

3단계에서 BT(`AlignDock`)로 승격하면 **aba 스택이 반드시 떠 있다**(복귀 중 도킹이므로).
그 순간 장치를 직접 여는 코드는 앞캠을 죽인다. 그래서 프레임 획득을 함수 하나 뒤에 가둔다:

```python
# 1단계 (이 레포, 단독 실행)
def get_frame():
    return _camera.read()

# 3단계 (aba 스택 위) — 이 함수만 갈아끼운다
def get_frame():
    return frame_tap.read("front")
```

상태기계 안에는 `camera.read()` 가 한 번도 등장하지 않는다. 이음매가 없으면 3단계에서
흩어진 호출을 전부 찾아 고쳐야 한다.

## 제어 상태기계

| phase | 진입 | 동작 | 이탈 |
|---|---|---|---|
| `SEARCH` | 시작 시 마커 없음 | 20° 펄스 회전 → 정지 → 재검출. 순서: 좌 20/40/60 → 중앙 → 우 20/40/60. 회전량은 **`/odom` yaw 피드백**으로 닫는다 | 검출 → `HOMING` / 1스윕 실패 → `ABORT(not_found)` |
| `HOMING` | 마커 보임, `z > 0.6m` | `ex` PID 조향 + 연속 전진. yaw 미사용 | `z ≤ 0.6m` → `AXIS_ALIGN` / 상실 → 아래 규칙 |
| `AXIS_ALIGN` | `z ≤ 0.6m` | yaw·축이탈 사용. 펄스 이동(0.10s 가고 멈춰 재검출) | `z ≤ 0.10m` → `DONE` / `z ≤ 0.25m` & 축 정렬됨 → `BLIND_PUSH` |
| `BLIND_PUSH` | 근접 도달 또는 근접 상실 | `/odom` 으로 `(마지막 z − 목표거리)` 만큼 전진 | 도달 → `DONE` |
| `DONE` / `ABORT` | — | 모터 0 | — |

**마커 상실은 마지막 관측 거리로 갈린다** — 같은 상실이라도 대응이 반대다.

| 마지막 `z` | 8프레임 상실 후 |
|---|---|
| > 0.25m | `SEARCH` — 정말 놓쳤다 |
| ≤ 0.25m | `BLIND_PUSH` — 가까워져 시야에서 빠진 것이다(정상) |

회전 후 정지(`turn_pause_s`)는 모션블러 회피다. 연속 회전은 마커를 시야에서 날린다.
시간 기반 회전을 쓰지 않는 이유: 같은 시간을 돌아도 바닥 재질·배터리 잔량에 따라 회전량이
달라진다(`aba_project` 의 회복 동작이 시간 기반이라 정확히 그 약점을 안고 있다).

### 상수 — 현장 검증값 전사

`aba_project/.../aruco_dock.py` 의 2026-07-06 실주행 튜닝값을 옮겨 적는다. 새로 지어내면
튜닝을 처음부터 다시 한다.

```
steer_kp 0.22 / ki 0.01 / kd 0.0     (:280-282, D 는 튐 방지로 0)
steer_deadband 0.012                  (:283)
steer_ang_max 0.08 / ang_min 0.025    (:279, :284)
steer_i_max 0.5                       (:285 anti-windup)
ex_lpf_alpha 0.45                     (:287)
turn_pause_s 0.40                     (:296)
move_pulse_s 0.10 / move_pause_s 0.90 (:313-314)
pose_kp_yaw 1.0 / pose_kp_lat 1.5     (:304-305)
pose_axis_tol_m 0.08                  (:307)
lost_grace 8프레임 / loop_hz 12        (:267, :318)
steer_sign +1                         (:58, 2026-07-06 현장 재검증)
```

`/cmd_vel` 로 나가므로 모터 출력%(`min_drive`)는 안 쓰고 속도(m/s, rad/s)로 대체한다:
`lin_homing 0.12`, `lin_pulse 0.08`, `ang_search 0.35`.

### 버리는 것

`skew` 기반 무보정 접근(캘리브가 있다), 후면 회전, nav2 프로세스 관리, 초음파 경로,
`turn_deg_per_s` 시간 추정. **파라미터 25개 이하 유지.**

### 실기에서만 정해지는 값

| 값 | 기본 | 정하는 법 |
|---|---|---|
| `steer_sign` | +1 | 로봇이 반대로 흐르면 -1. 소프트웨어로 판단 불가 |
| `axis_gate_m` | 0.6 | 노트북 관찰로 yaw 안정 거리 확인 후 |
| `front_offset_m` | 0.0 | 카메라가 최전방에 가깝다는 확인. `detect` 출력과 자로 잰 값 비교 |
| 사전 이름 | `DICT_5X5_100` | `detect --scan-dicts` 로 실물 확정 |

### 안전장치

| 상황 | 대응 |
|---|---|
| 마커 상실 | 8프레임 유예 후 위 분기 |
| 전체 시간 초과 | 60초 → `ABORT(timeout)` |
| 전진 무진전 2.5초 | `ABORT(blocked)` |
| 정렬 미수렴 5초 | `ABORT(align_stall)` |
| 원본 `/scan` 전방 6cm 미만 | 즉시 정지 `ABORT(scan_guard)` |
| Ctrl-C | 즉시 모터 0 |
| SEARCH 1스윕 실패 | 무한 회전 금지, `ABORT(not_found)` |

`scan_filtered` 를 쓰지 않는다 — 그쪽은 `min_range 0.05` 로 근접 구간이 제거된다.

## 캘리브레이션

`config/camera/` 에 두 파일을 둔다(`aba_project` 에서 복사):

| 슬롯 | 파일 | fx | cx |
|---|---|---|---|
| front | `picam_640x480_rot180.npz` | 609.2 | 278.2 |
| back | `usb_640x480.npz` | 675.0 | 255.9 |

회전 보정이 다른 캘리브를 물리면 **거리는 그럴듯한데 정렬만 한쪽으로 흐른다**(비회전본은
cx 360.8 로 80px 차이). 슬롯 이름이 곧 K 선택 키가 되게 묶어 어긋날 여지를 없앤다.

## OpenCV 버전

`cv2.aruco` API 가 4.6 과 4.7+ 에서 다르다(`ArucoDetector` 는 4.7+). 이 개발 머신만 해도
시스템 python3 는 4.6.0, 가상환경은 5.0.0 이다. 검출 코드는 **양쪽을 모두 지원**한다.

## 범위 밖

- BT(`FaceParking`/`AlignDock`) 배선 — 3단계
- 뒷캠 후진 정렬 — 3단계. `slot` 파라미터 자리만 둔다
- 마커 여러 장 경로 주행, ChArUco, 중첩 마커
- 관제 화면 연동
- `aba_project` 쪽 코드 수정(생프레임 탭·`cmd_bridge` 등) — 이번 단독 실행에서는 불필요
