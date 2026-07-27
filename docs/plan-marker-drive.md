# 마커 주행(ArUco Marker Drive) 1단계 Implementation Plan

> ⚠️ **이 문서는 착수 시점의 계획이며, 구현 도중 여러 결정이 뒤집혔다.**
> 현재 동작의 정본은 `README.md` 와 `docs/design-marker-drive.md`(“구현하며 확정·수정된 것”
> 절)이다. 특히 축 정렬 제어식의 부호, 무시각 구간 거리 측정, 정렬 판정, 센서 신선도는
> 여기 적힌 코드와 다르다. 이 파일은 어떤 순서로 무엇을 세웠는지를 남기는 이력이다.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** ArUco 마커 한 장을 보고 마커 앞 10cm(로봇 최전방 기준)에 정지하는 주행 코어를, 다른 스택에 의존하지 않는 단독 실행 CLI 로 만든다.

**Architecture:** 판단 로직은 카메라·모터·시계·ROS 를 모르는 순수 상태기계(`approach.py`)에 격리한다. 프레임 획득은 `camera.get_frame()` 이음매 하나 뒤에 가둬, 나중에 다른 스택 위로 승격할 때 그 함수만 바뀌게 한다. 모터는 `/cmd_vel` 발행만 하고, 회전량·전진량은 `/odom` 피드백으로 닫는다.

**Tech Stack:** Python 3.12(시스템), OpenCV(`cv2.aruco` — 4.6 과 4.7+ 양쪽 지원), NumPy, rclpy(ROS2 Jazzy), pytest 7.4.4, picamera2(CSI 카메라, 시스템 패키지)

## Global Constraints

- 레포 루트: `/home/ane/personal_repo/arte_aurcomaker_move`. 브랜치 `main`, remote `git@github.com:dlrkdxor0821/arte_aurcomaker_move.git`.
- **다른 저장소의 코드를 임포트하지 않는다.** `aba_project` 는 참고 대상일 뿐이며, 이 레포는 단독으로 내려받아 돌아야 한다.
- 마커: 5x5 계열, 대상 ID **1**, 한 변 **0.07 m**. 기본 사전 `DICT_5X5_100`(실물 확정은 `detect --scan-dicts` 로).
- 정지 목표: 마커에서 **로봇 최전방까지 0.10 m**. 판정식 `z_m <= stop_m + front_offset_m`, `front_offset_m` 기본 **0.0**.
- 모터 명령은 **`/cmd_vel`(geometry_msgs/Twist) 발행만**. `sudo` 를 요구하는 하드웨어 직접 제어를 쓰지 않는다.
- 근접 안전 감시는 **원본 `/scan`**. 필터된 스캔은 근접 구간이 제거돼 있어 쓰지 않는다.
- 상태기계(`approach.py`) 안에 카메라·모터·ROS 호출이 **한 번도 등장하지 않는다.**
- 모든 튜닝값은 CLI 플래그로 노출한다.
- 테스트는 시스템 python3 로 돈다: `python3 -m pytest marker/tests/ -v` (레포 루트에서). 가상환경을 만들지 않는다 — 로봇의 `picamera2` 는 시스템 패키지라 venv 에서 안 잡힌다.
- `cv2.aruco` 는 **4.6 과 4.7+ 를 모두 지원**한다(개발 머신 시스템 python3 = 4.6.0). `ArucoDetector`·`generateImageMarker` 는 4.7+ 에만 있다.
- git 커밋은 각 Task 끝에서만. merge·push 는 하지 않는다(사용자 몫).

## File Structure

```
arte_aurcomaker_move/
  README.md                     사용법·전제조건 (Task 7 에서 완성)
  .gitignore
  config/camera/
    picam_640x480_rot180.npz    앞캠(picam, rotate 180) — aba_project 에서 복사
    usb_640x480.npz             뒷캠(USB) — 3단계용, 지금은 미사용
  marker/
    __init__.py
    types.py                    MarkerObs, Cmd — 값 전달만
    config.py                   MarkerDriveConfig — 튜닝 상수 기본값
    approach.py                 MarkerApproach — 순수 상태기계 (핵심)
    detect.py                   detect_marker, scan_dicts — 프레임 → MarkerObs
    calib.py                    load_calib — 슬롯 → (K, dist)
    camera.py                   open_camera / get_frame — ★ 프레임 획득 이음매
    odom.py                     OdomTracker — /odom → 누적 yaw, 누적 전진거리
    scan.py                     ScanWatch — 원본 /scan 전방 최소거리
    drive.py                    CLI 진입점
    watch.py                    노트북 관찰
    tests/
      __init__.py
      test_config.py
      test_approach.py
      test_detect.py
  marker-drive.sh               로봇용 런처
  marker-watch.sh               노트북용 런처
  docs/                         design / prd / plan (이미 있음)
```

---

### Task 1: 레포 뼈대, 값 타입, 설정

**Files:**
- Create: `.gitignore`, `marker/__init__.py`, `marker/tests/__init__.py`
- Create: `marker/types.py`, `marker/config.py`
- Create: `config/camera/picam_640x480_rot180.npz`, `config/camera/usb_640x480.npz` (복사)
- Test: `marker/tests/test_config.py`

**Interfaces:**
- Consumes: 없음
- Produces:
  - `MarkerObs(marker_id: int, ex: float, z_m: float, x_m: float, yaw_deg: float, lateral_m: float, size_frac: float)` — frozen dataclass
  - `Cmd(linear: float, angular: float, phase: str, done: bool, reason: str)` — frozen dataclass
  - `MarkerDriveConfig` — frozen dataclass, `clamped() -> MarkerDriveConfig`

- [ ] **Step 1: 캘리브 파일 복사와 .gitignore**

```bash
cd /home/ane/personal_repo/arte_aurcomaker_move
mkdir -p config/camera marker/tests
cp /home/ane/personal_repo/aba_project/config/camera/picam_640x480_rot180.npz config/camera/
cp /home/ane/personal_repo/aba_project/config/camera/usb_640x480.npz config/camera/
python3 -c "
import numpy as np
for n in ['picam_640x480_rot180','usb_640x480']:
    d = np.load(f'config/camera/{n}.npz'); K = d['camera_matrix']
    print(n, 'fx=%.1f cx=%.1f' % (K[0,0], K[0,2]))
"
```
Expected: `picam_640x480_rot180 fx=609.2 cx=278.2` / `usb_640x480 fx=675.0 cx=255.9`

`.gitignore`:

```
__pycache__/
*.pyc
.pytest_cache/
*.egg-info/
```

`marker/__init__.py`, `marker/tests/__init__.py` — 빈 파일.

- [ ] **Step 2: 실패하는 테스트 작성**

`marker/tests/test_config.py`:

```python
from marker.config import MarkerDriveConfig
from marker.types import Cmd, MarkerObs


def test_defaults_match_field_verified_constants():
    c = MarkerDriveConfig()
    assert c.steer_kp == 0.22
    assert c.steer_ki == 0.01
    assert c.steer_kd == 0.0
    assert c.steer_deadband == 0.012
    assert c.steer_ang_max == 0.08
    assert c.steer_sign == 1.0
    assert c.axis_gate_m == 0.6
    assert c.stop_m == 0.10
    assert c.front_offset_m == 0.0
    assert c.marker_len_m == 0.07
    assert c.marker_id == 1
    assert c.dict_name == "DICT_5X5_100"


def test_clamped_forces_sane_ranges():
    c = MarkerDriveConfig(steer_kp=-5.0, loop_hz=999.0, search_step_deg=0.0).clamped()
    assert c.steer_kp == 0.0
    assert c.loop_hz == 30.0
    assert c.search_step_deg == 1.0


def test_clamped_covers_timing_fields_too():
    """일부만 덮으면 안 덮인 필드로 상태기계를 마비시킬 수 있다.

    move_pulse_s=-1 이면 펄스 종료 시각이 과거라 전진 명령이 영영 안 나간다.
    """
    c = MarkerDriveConfig(move_pulse_s=-1.0, timeout_s=0.0, align_stall_s=-3.0,
                          ex_lpf_alpha=9.0, pose_yaw_tol_deg=0.0).clamped()
    assert c.move_pulse_s >= 0.02
    assert c.timeout_s >= 1.0
    assert c.align_stall_s >= 0.2
    assert c.ex_lpf_alpha <= 1.0
    assert c.pose_yaw_tol_deg >= 1.0


def test_steer_sign_is_normalized_to_plus_or_minus_one():
    assert MarkerDriveConfig(steer_sign=-0.3).clamped().steer_sign == -1.0
    assert MarkerDriveConfig(steer_sign=4.0).clamped().steer_sign == 1.0


def test_value_types_are_frozen():
    obs = MarkerObs(marker_id=1, ex=0.0, z_m=1.0, x_m=0.0,
                    yaw_deg=0.0, lateral_m=0.0, size_frac=0.1)
    cmd = Cmd(linear=0.0, angular=0.0, phase="SEARCH", done=False, reason="")
    for frozen, field in ((obs, "z_m"), (cmd, "linear")):
        try:
            setattr(frozen, field, 9.9)
        except Exception as exc:
            assert "frozen" in str(exc).lower()
        else:
            raise AssertionError(f"{type(frozen).__name__} 이 frozen 이 아니다")
```

- [ ] **Step 3: 테스트 실패 확인**

Run: `cd /home/ane/personal_repo/arte_aurcomaker_move && python3 -m pytest marker/tests/test_config.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'marker.config'`

- [ ] **Step 4: 구현**

`marker/types.py`:

```python
"""마커 주행이 주고받는 값 두 개. 로직 없음."""
from dataclasses import dataclass


@dataclass(frozen=True)
class MarkerObs:
    """한 프레임에서 뽑은 마커 관측값.

    ex        : 화면 중앙 대비 좌우 오차. -1(왼쪽 끝) ~ +1(오른쪽 끝)
    z_m       : 카메라에서 마커까지 거리(m)
    x_m       : 카메라 광축 기준 마커의 좌우 오프셋(m)
    yaw_deg   : 마커 정면각(0 = 마커를 정면으로 마주봄)
    lateral_m : 마커 법선축에서 로봇이 벗어난 거리(m)
    size_frac : 마커 한 변이 프레임 폭에서 차지하는 비율
    """
    marker_id: int
    ex: float
    z_m: float
    x_m: float
    yaw_deg: float
    lateral_m: float
    size_frac: float


@dataclass(frozen=True)
class Cmd:
    """상태기계가 내는 한 틱의 명령."""
    linear: float
    angular: float
    phase: str
    done: bool
    reason: str
```

`marker/config.py`:

```python
"""튜닝 상수 기본값.

게인·데드밴드·펄스 길이는 aba_project 의 기존 도킹 구현(aruco_dock.py)에서
2026-07-06 실주행으로 맞춘 값을 옮겨 적은 것이다. 새로 지어낸 값이 아니므로
근거 없이 바꾸지 않는다. 그 저장소를 임포트하지는 않는다(단독 실행 원칙).
"""
from dataclasses import dataclass, replace


def _clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


@dataclass(frozen=True)
class MarkerDriveConfig:
    # --- 마커 ---
    marker_id: int = 1
    marker_len_m: float = 0.07
    dict_name: str = "DICT_5X5_100"

    # --- 정지 조건 ---
    stop_m: float = 0.10            # 마커 ~ 로봇 최전방 목표 거리
    front_offset_m: float = 0.0     # 카메라 렌즈 ~ 로봇 최전방
    scan_guard_m: float = 0.06      # 원본 /scan 전방이 이보다 가까우면 즉시 정지

    # --- 조향 PID (aruco_dock.py:279-285 전사) ---
    steer_kp: float = 0.22
    steer_ki: float = 0.01
    steer_kd: float = 0.0
    steer_deadband: float = 0.012
    steer_ang_max: float = 0.08
    steer_ang_min: float = 0.025
    steer_i_max: float = 0.5
    steer_sign: float = 1.0         # 반대로 돌면 -1. 소프트웨어로 판단 불가

    # --- 근거리 축 정렬 (aruco_dock.py:304-307 전사) ---
    axis_gate_m: float = 0.6        # 이 거리 안에서만 yaw 를 신뢰한다
    pose_kp_yaw: float = 1.0
    pose_kp_lat: float = 1.5
    pose_axis_tol_m: float = 0.08
    pose_yaw_tol_deg: float = 8.0   # 정렬 완료 판정에 yaw 도 포함한다.
    #   lateral 만 보면 yaw 가 30° 틀어진 채로 무시각 구간에 진입한다.

    # --- 속도 (/cmd_vel 이라 m/s, rad/s) ---
    lin_homing: float = 0.12
    lin_pulse: float = 0.08
    ang_search: float = 0.35

    # --- 펄스 (aruco_dock.py:296, 313-314 전사) ---
    move_pulse_s: float = 0.10
    move_pause_s: float = 0.90
    turn_pause_s: float = 0.40      # 회전 후 정지·재검출(모션블러 회피)

    # --- 탐색 ---
    search_step_deg: float = 20.0
    search_span_deg: float = 60.0   # 중앙 기준 좌우 각각의 최대 각
    search_tol_deg: float = 2.0     # 목표 각 도달 판정 여유
    search_step_timeout_s: float = 3.0
    #   한 스텝을 이 시간 안에 못 돌면 회전이 실패한 것이다(바퀴 헛돎·odom 정지).
    #   없으면 전역 타임아웃까지 헛돌며 원인을 'timeout' 으로 오분류한다.

    # --- 필터·유예 ---
    ex_lpf_alpha: float = 0.45
    lost_grace: int = 8             # 프레임
    lost_near_m: float = 0.25       # 마지막 관측이 이보다 가까우면 상실=근접진입

    # --- 안전 ---
    timeout_s: float = 60.0
    no_progress_s: float = 2.5
    align_stall_s: float = 5.0
    loop_hz: float = 12.0

    def clamped(self) -> "MarkerDriveConfig":
        """모든 수치 필드를 안전 범위로 강제한다.

        일부만 덮으면 덮이지 않은 필드로 상태기계를 마비시킬 수 있다.
        예: move_pulse_s=-1 이면 펄스 종료 시각이 과거라 전진 명령이 영영 안 나간다.
        """
        return replace(
            self,
            marker_len_m=_clamp(self.marker_len_m, 0.005, 1.0),
            steer_kp=_clamp(self.steer_kp, 0.0, 1.0),
            steer_ki=_clamp(self.steer_ki, 0.0, 0.5),
            steer_kd=_clamp(self.steer_kd, 0.0, 1.0),
            steer_deadband=_clamp(self.steer_deadband, 0.0, 0.1),
            steer_ang_max=_clamp(self.steer_ang_max, 0.0, 0.3),
            steer_ang_min=_clamp(self.steer_ang_min, 0.0, 0.1),
            steer_i_max=_clamp(self.steer_i_max, 0.0, 2.0),
            steer_sign=1.0 if self.steer_sign >= 0 else -1.0,
            axis_gate_m=_clamp(self.axis_gate_m, 0.15, 2.0),
            pose_kp_yaw=_clamp(self.pose_kp_yaw, 0.0, 5.0),
            pose_kp_lat=_clamp(self.pose_kp_lat, 0.0, 5.0),
            pose_axis_tol_m=_clamp(self.pose_axis_tol_m, 0.005, 0.4),
            pose_yaw_tol_deg=_clamp(self.pose_yaw_tol_deg, 1.0, 45.0),
            stop_m=_clamp(self.stop_m, 0.02, 1.0),
            front_offset_m=_clamp(self.front_offset_m, 0.0, 0.5),
            scan_guard_m=_clamp(self.scan_guard_m, 0.0, 0.5),
            lin_homing=_clamp(self.lin_homing, 0.02, 0.25),
            lin_pulse=_clamp(self.lin_pulse, 0.02, 0.25),
            ang_search=_clamp(self.ang_search, 0.05, 1.0),
            move_pulse_s=_clamp(self.move_pulse_s, 0.02, 1.0),
            move_pause_s=_clamp(self.move_pause_s, 0.0, 3.0),
            turn_pause_s=_clamp(self.turn_pause_s, 0.0, 3.0),
            search_step_deg=_clamp(self.search_step_deg, 1.0, 90.0),
            search_span_deg=_clamp(self.search_span_deg, 5.0, 180.0),
            search_tol_deg=_clamp(self.search_tol_deg, 0.5, 15.0),
            search_step_timeout_s=_clamp(self.search_step_timeout_s, 0.5, 20.0),
            ex_lpf_alpha=_clamp(self.ex_lpf_alpha, 0.05, 1.0),
            lost_grace=int(_clamp(self.lost_grace, 1, 60)),
            lost_near_m=_clamp(self.lost_near_m, 0.02, 1.0),
            timeout_s=_clamp(self.timeout_s, 1.0, 600.0),
            no_progress_s=_clamp(self.no_progress_s, 0.2, 20.0),
            align_stall_s=_clamp(self.align_stall_s, 0.2, 60.0),
            loop_hz=_clamp(self.loop_hz, 2.0, 30.0),
        )
```

- [ ] **Step 5: 테스트 통과 확인**

Run: `cd /home/ane/personal_repo/arte_aurcomaker_move && python3 -m pytest marker/tests/test_config.py -v`
Expected: PASS (4 passed)

- [ ] **Step 6: 커밋**

```bash
cd /home/ane/personal_repo/arte_aurcomaker_move
git add .gitignore config/camera marker/__init__.py marker/types.py marker/config.py marker/tests/
git commit -m "feat: add marker drive value types, tuned config defaults and camera calibration"
```

---

### Task 2: 상태기계 — 탐색과 원거리 접근

**Files:**
- Create: `marker/approach.py`
- Test: `marker/tests/test_approach.py`

**Interfaces:**
- Consumes: `MarkerObs`, `Cmd`, `MarkerDriveConfig` (Task 1)
- Produces:
  - `MarkerApproach(cfg: MarkerDriveConfig)`
  - `MarkerApproach.step(obs: MarkerObs | None, *, yaw_deg: float, travel_m: float, front_m: float | None, now_s: float) -> Cmd`
    - `yaw_deg`: `/odom` 누적 방위각(도, 좌회전 +) · `travel_m`: 누적 전진거리(m) · `front_m`: 원본 `/scan` 전방 최소거리(없으면 `None`) · `now_s`: 단조 증가 초
  - `MarkerApproach.phase` — `SEARCH`/`HOMING`/`AXIS_ALIGN`/`BLIND_PUSH`/`DONE`/`ABORT`

- [ ] **Step 1: 실패하는 테스트 작성**

`marker/tests/test_approach.py`:

```python
import pytest

from marker.approach import MarkerApproach
from marker.config import MarkerDriveConfig
from marker.types import MarkerObs


def obs(z, ex=0.0, yaw=0.0, lat=0.0):
    return MarkerObs(marker_id=1, ex=ex, z_m=z, x_m=0.0,
                     yaw_deg=yaw, lateral_m=lat, size_frac=0.1)


def test_search_turns_left_first():
    m = MarkerApproach(MarkerDriveConfig())
    c = m.step(None, yaw_deg=0.0, travel_m=0.0, front_m=None, now_s=0.0)
    assert c.phase == "SEARCH"
    assert c.angular > 0
    assert c.linear == 0.0


def test_search_stops_turning_at_step_target():
    m = MarkerApproach(MarkerDriveConfig(search_step_deg=20.0, turn_pause_s=0.4))
    m.step(None, yaw_deg=0.0, travel_m=0.0, front_m=None, now_s=0.0)
    c = m.step(None, yaw_deg=20.0, travel_m=0.0, front_m=None, now_s=0.5)
    assert c.angular == 0.0        # 목표 각 도달 → 멈춰서 본다(모션블러 회피)


def test_search_aborts_after_full_sweep():
    m = MarkerApproach(MarkerDriveConfig(search_step_deg=20.0, search_span_deg=60.0,
                                         turn_pause_s=0.0))
    yaw, t, c = 0.0, 0.0, None
    for _ in range(500):
        c = m.step(None, yaw_deg=yaw, travel_m=0.0, front_m=None, now_s=t)
        if c.done:
            break
        yaw += 5.0 if c.angular > 0 else (-5.0 if c.angular < 0 else 0.0)
        t += 0.1
    assert c.phase == "ABORT"
    assert c.reason == "not_found"


def test_marker_found_switches_to_homing():
    m = MarkerApproach(MarkerDriveConfig())
    c = m.step(obs(1.5), yaw_deg=0.0, travel_m=0.0, front_m=None, now_s=0.0)
    assert c.phase == "HOMING"
    assert c.linear > 0


def test_homing_steers_proportionally_to_ex():
    small = MarkerApproach(MarkerDriveConfig()).step(
        obs(1.5, ex=0.10), yaw_deg=0.0, travel_m=0.0, front_m=None, now_s=0.0)
    big = MarkerApproach(MarkerDriveConfig()).step(
        obs(1.5, ex=0.40), yaw_deg=0.0, travel_m=0.0, front_m=None, now_s=0.0)
    assert abs(big.angular) > abs(small.angular)


def test_homing_ignores_flipped_yaw_beyond_gate():
    a = MarkerApproach(MarkerDriveConfig()).step(
        obs(1.5, ex=0.2, yaw=+170.0), yaw_deg=0.0, travel_m=0.0, front_m=None, now_s=0.0)
    b = MarkerApproach(MarkerDriveConfig()).step(
        obs(1.5, ex=0.2, yaw=-170.0), yaw_deg=0.0, travel_m=0.0, front_m=None, now_s=0.0)
    assert a.angular == pytest.approx(b.angular)


def test_homing_respects_deadband():
    m = MarkerApproach(MarkerDriveConfig(steer_deadband=0.05))
    c = m.step(obs(1.5, ex=0.01), yaw_deg=0.0, travel_m=0.0, front_m=None, now_s=0.0)
    assert c.angular == 0.0


def test_steer_sign_flips_direction():
    plus = MarkerApproach(MarkerDriveConfig(steer_sign=1.0)).step(
        obs(1.5, ex=0.3), yaw_deg=0.0, travel_m=0.0, front_m=None, now_s=0.0)
    minus = MarkerApproach(MarkerDriveConfig(steer_sign=-1.0)).step(
        obs(1.5, ex=0.3), yaw_deg=0.0, travel_m=0.0, front_m=None, now_s=0.0)
    assert plus.angular == pytest.approx(-minus.angular)


def test_gate_switches_to_axis_align():
    m = MarkerApproach(MarkerDriveConfig(axis_gate_m=0.6))
    m.step(obs(0.9), yaw_deg=0.0, travel_m=0.0, front_m=None, now_s=0.0)
    c = m.step(obs(0.5), yaw_deg=0.0, travel_m=0.0, front_m=None, now_s=0.1)
    assert c.phase == "AXIS_ALIGN"


def test_timeout_aborts():
    m = MarkerApproach(MarkerDriveConfig(timeout_s=1.0))
    m.step(obs(1.5), yaw_deg=0.0, travel_m=0.0, front_m=None, now_s=0.0)
    c = m.step(obs(1.5), yaw_deg=0.0, travel_m=0.0, front_m=None, now_s=2.0)
    assert c.phase == "ABORT" and c.reason == "timeout"
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `cd /home/ane/personal_repo/arte_aurcomaker_move && python3 -m pytest marker/tests/test_approach.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'marker.approach'`

- [ ] **Step 3: 구현**

`marker/approach.py`:

```python
"""마커 주행 판단 로직 — 순수 상태기계.

카메라·모터·시계·ROS 를 모른다. 입력은 관측값과 센서 수치, 출력은 명령뿐이다.
그래서 하드웨어 없이 전체 판단을 테스트할 수 있고, 나중에 다른 스택 위로 승격할 때
이 파일은 손대지 않는다.
"""
from .config import MarkerDriveConfig
from .types import Cmd, MarkerObs


def _sweep_targets(step_deg: float, span_deg: float) -> list[float]:
    """탐색 목표 각도 순서: 좌로 훑고 중앙 복귀 후 우로 훑는다."""
    n = max(1, int(round(span_deg / step_deg)))
    left = [step_deg * (i + 1) for i in range(n)]
    right = [-step_deg * (i + 1) for i in range(n)]
    return left + [0.0] + right


class MarkerApproach:
    def __init__(self, cfg: MarkerDriveConfig):
        self.cfg = cfg.clamped()
        self.phase = "SEARCH"
        self._targets = _sweep_targets(self.cfg.search_step_deg, self.cfg.search_span_deg)
        self._target_i = 0
        self._yaw0 = None            # 탐색 기준이 되는 시작 방위각
        self._settle_until = 0.0     # 회전 후 재검출 대기 종료 시각
        self._ex_f = None            # ex 저역통과 상태
        self._i_acc = 0.0            # 조향 적분 누적
        self._prev_ex = 0.0
        self._t0 = None              # 전체 시작 시각

    # ---- 내부 헬퍼 -------------------------------------------------------
    def _stop(self, reason: str, phase: str) -> Cmd:
        self.phase = phase
        return Cmd(0.0, 0.0, phase, phase in ("DONE", "ABORT"), reason)

    def _filter_ex(self, ex: float) -> float:
        a = self.cfg.ex_lpf_alpha
        self._ex_f = ex if self._ex_f is None else (a * ex + (1 - a) * self._ex_f)
        return self._ex_f

    def _steer(self, ex: float, dt: float) -> float:
        """ex 기반 조향 각속도.  angular = -sign * (kp*e + ki*∫e + kd*de/dt)"""
        c = self.cfg
        e = self._filter_ex(ex)
        if abs(e) < c.steer_deadband:
            self._i_acc = 0.0
            self._prev_ex = e
            return 0.0
        self._i_acc = max(-c.steer_i_max, min(c.steer_i_max, self._i_acc + e * dt))
        d = (e - self._prev_ex) / dt if dt > 0 else 0.0
        self._prev_ex = e
        u = c.steer_kp * e + c.steer_ki * self._i_acc + c.steer_kd * d
        mag = min(c.steer_ang_max, max(c.steer_ang_min, abs(u)))
        return -c.steer_sign * mag * (1.0 if u >= 0 else -1.0)

    # ---- 단계 ------------------------------------------------------------
    def _do_search(self, yaw_deg: float, now_s: float) -> Cmd:
        c = self.cfg
        if self._yaw0 is None:
            self._yaw0 = yaw_deg
        if self._target_i >= len(self._targets):
            return self._stop("not_found", "ABORT")
        if now_s < self._settle_until:
            return Cmd(0.0, 0.0, "SEARCH", False, "settle")
        err = self._targets[self._target_i] - (yaw_deg - self._yaw0)
        if abs(err) <= c.search_tol_deg:          # 도달 → 멈춰서 본다
            self._target_i += 1
            self._settle_until = now_s + c.turn_pause_s
            return Cmd(0.0, 0.0, "SEARCH", False, "look")
        return Cmd(0.0, c.ang_search if err > 0 else -c.ang_search,
                   "SEARCH", False, "sweep")

    def _do_homing(self, o: MarkerObs, dt: float) -> Cmd:
        return Cmd(self.cfg.lin_homing, self._steer(o.ex, dt), "HOMING", False, "approach")

    # ---- 진입점 ----------------------------------------------------------
    def step(self, obs: MarkerObs | None, *, yaw_deg: float, travel_m: float,
             front_m: float | None, now_s: float) -> Cmd:
        c = self.cfg
        if self._t0 is None:
            self._t0 = now_s
        if self.phase in ("DONE", "ABORT"):
            return Cmd(0.0, 0.0, self.phase, True, "finished")
        if now_s - self._t0 > c.timeout_s:
            return self._stop("timeout", "ABORT")
        if front_m is not None and front_m < c.scan_guard_m:
            return self._stop("scan_guard", "ABORT")
        dt = 1.0 / c.loop_hz

        if obs is not None and obs.marker_id == c.marker_id:
            if self.phase in ("SEARCH", "HOMING"):
                self.phase = "AXIS_ALIGN" if obs.z_m <= c.axis_gate_m else "HOMING"
            if self.phase == "HOMING":
                return self._do_homing(obs, dt)
            return Cmd(0.0, 0.0, self.phase, False, "hold")
        if self.phase == "SEARCH":
            return self._do_search(yaw_deg, now_s)
        return Cmd(0.0, 0.0, self.phase, False, "hold")
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `cd /home/ane/personal_repo/arte_aurcomaker_move && python3 -m pytest marker/tests/test_approach.py -v`
Expected: PASS (10 passed)

- [ ] **Step 5: 커밋**

```bash
cd /home/ane/personal_repo/arte_aurcomaker_move
git add marker/approach.py marker/tests/test_approach.py
git commit -m "feat: add search sweep and homing phases of approach state machine"
```

---

### Task 3: 상태기계 — 축 정렬, 무시각 전진, 상실 처리

**Files:**
- Modify: `marker/approach.py`
- Modify: `marker/tests/test_approach.py`

**Interfaces:**
- Consumes: `MarkerApproach.step()` (Task 2) — 시그니처 불변
- Produces: 완성된 전이. 중단 이유는 `not_found` / `timeout` / `scan_guard` / `blocked` / `align_stall` 중 하나.

- [ ] **Step 1: 실패하는 테스트 추가**

`marker/tests/test_approach.py` 끝에 이어붙인다:

```python
def _to_axis_align(cfg=None):
    m = MarkerApproach(cfg or MarkerDriveConfig())
    m.step(obs(0.9), yaw_deg=0.0, travel_m=0.0, front_m=None, now_s=0.0)
    m.step(obs(0.5), yaw_deg=0.0, travel_m=0.0, front_m=None, now_s=0.1)
    return m


def test_axis_align_corrects_more_when_skewed():
    straight = _to_axis_align().step(obs(0.5, yaw=0.0, lat=0.0), yaw_deg=0.0,
                                     travel_m=0.0, front_m=None, now_s=0.2)
    skewed = _to_axis_align().step(obs(0.5, yaw=15.0, lat=0.12), yaw_deg=0.0,
                                   travel_m=0.0, front_m=None, now_s=0.2)
    assert abs(skewed.angular) > abs(straight.angular)


def test_axis_align_moves_in_pulses():
    m = _to_axis_align(MarkerDriveConfig(move_pulse_s=0.1, move_pause_s=0.9))
    moving = m.step(obs(0.4), yaw_deg=0.0, travel_m=0.0, front_m=None, now_s=0.2)
    resting = m.step(obs(0.4), yaw_deg=0.0, travel_m=0.0, front_m=None, now_s=0.35)
    assert moving.linear > 0
    assert resting.linear == 0.0


def test_reaching_stop_distance_while_visible_is_done():
    m = _to_axis_align(MarkerDriveConfig(stop_m=0.10, front_offset_m=0.0))
    c = m.step(obs(0.09), yaw_deg=0.0, travel_m=0.0, front_m=None, now_s=0.2)
    assert c.done and c.phase == "DONE" and c.reason == "reached"


def test_lost_far_returns_to_search():
    m = MarkerApproach(MarkerDriveConfig(lost_grace=3, lost_near_m=0.25))
    m.step(obs(0.9), yaw_deg=0.0, travel_m=0.0, front_m=None, now_s=0.0)
    c = None
    for i in range(3):
        c = m.step(None, yaw_deg=0.0, travel_m=0.0, front_m=None, now_s=0.1 * (i + 1))
    assert c.phase == "SEARCH"


def test_lost_near_enters_blind_push():
    m = MarkerApproach(MarkerDriveConfig(lost_grace=3, lost_near_m=0.25))
    m.step(obs(0.5), yaw_deg=0.0, travel_m=0.0, front_m=None, now_s=0.0)
    m.step(obs(0.18), yaw_deg=0.0, travel_m=0.0, front_m=None, now_s=0.1)
    c = None
    for i in range(3):
        c = m.step(None, yaw_deg=0.0, travel_m=0.0, front_m=None, now_s=0.2 + 0.1 * i)
    assert c.phase == "BLIND_PUSH"


def test_blind_push_travels_remaining_then_done():
    m = MarkerApproach(MarkerDriveConfig(lost_grace=1, lost_near_m=0.25,
                                         stop_m=0.10, front_offset_m=0.0))
    m.step(obs(0.5), yaw_deg=0.0, travel_m=0.0, front_m=None, now_s=0.0)
    m.step(obs(0.16), yaw_deg=0.0, travel_m=0.0, front_m=None, now_s=0.1)
    c = m.step(None, yaw_deg=0.0, travel_m=0.0, front_m=None, now_s=0.2)
    assert c.phase == "BLIND_PUSH" and c.linear > 0
    c = m.step(None, yaw_deg=0.0, travel_m=0.03, front_m=None, now_s=0.3)
    assert not c.done                       # 0.06 중 0.03 만 갔다
    c = m.step(None, yaw_deg=0.0, travel_m=0.06, front_m=None, now_s=0.4)
    assert c.done and c.phase == "DONE"


def test_blind_push_blocked_when_no_progress():
    m = MarkerApproach(MarkerDriveConfig(lost_grace=1, lost_near_m=0.25, no_progress_s=0.5))
    m.step(obs(0.5), yaw_deg=0.0, travel_m=0.0, front_m=None, now_s=0.0)
    m.step(obs(0.18), yaw_deg=0.0, travel_m=0.0, front_m=None, now_s=0.1)
    c = None
    for i in range(10):
        c = m.step(None, yaw_deg=0.0, travel_m=0.0, front_m=None, now_s=0.2 + 0.1 * i)
    assert c.phase == "ABORT" and c.reason == "blocked"


def test_align_stall_aborts():
    m = _to_axis_align(MarkerDriveConfig(align_stall_s=0.5, pose_axis_tol_m=0.08))
    c = None
    for i in range(20):
        c = m.step(obs(0.18, lat=0.15), yaw_deg=0.0, travel_m=0.0,
                   front_m=None, now_s=0.2 + 0.1 * i)
    assert c.phase == "ABORT" and c.reason == "align_stall"


def test_scan_guard_stops_immediately():
    c = _to_axis_align().step(obs(0.3), yaw_deg=0.0, travel_m=0.0,
                              front_m=0.03, now_s=0.5)
    assert c.done and c.reason == "scan_guard"
    assert c.linear == 0.0 and c.angular == 0.0
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `cd /home/ane/personal_repo/arte_aurcomaker_move && python3 -m pytest marker/tests/test_approach.py -v`
Expected: FAIL — 새 테스트 9개 실패(`AXIS_ALIGN` 이 명령을 안 내고 `BLIND_PUSH` 가 없다)

- [ ] **Step 3: 구현**

`marker/approach.py` 의 `__init__` 마지막에 상태 필드를 추가한다:

```python
        self._lost = 0               # 연속 상실 프레임 수
        self._last_z = None          # 마지막으로 본 마커 거리
        self._pulse_until = 0.0      # 펄스 전진 종료 시각
        self._pulse_next = 0.0       # 다음 펄스 시작 가능 시각
        self._blind_target = None    # 무시각 전진 목표 거리(m)
        self._blind_travel0 = None   # 무시각 전진 시작 시점 누적거리
        self._progress_ref = None    # (travel_m, 시각) — 무진전 판정 기준
        self._align_since = None     # 축 정렬 시작 시각
```

`_do_homing` 아래에 단계 셋을 추가한다:

```python
    def _do_axis_align(self, o: MarkerObs, now_s: float) -> Cmd:
        c = self.cfg
        if self._align_since is None:
            self._align_since = now_s
        if o.z_m <= c.stop_m + c.front_offset_m:
            return self._stop("reached", "DONE")
        off_axis = abs(o.lateral_m) > c.pose_axis_tol_m
        if off_axis and now_s - self._align_since > c.align_stall_s:
            return self._stop("align_stall", "ABORT")
        # heading(yaw) 과 lateral 을 함께 줄여 마커 법선축에 올라탄다
        u = c.pose_kp_yaw * (o.yaw_deg / 90.0) + c.pose_kp_lat * o.lateral_m
        ang = -c.steer_sign * max(-c.steer_ang_max, min(c.steer_ang_max, u))
        if not off_axis and o.z_m <= c.lost_near_m:
            self._enter_blind(o.z_m, None)
            return Cmd(0.0, 0.0, "BLIND_PUSH", False, "near")
        if now_s >= self._pulse_next:
            self._pulse_until = now_s + c.move_pulse_s
            self._pulse_next = now_s + c.move_pulse_s + c.move_pause_s
        lin = c.lin_pulse if now_s < self._pulse_until else 0.0
        return Cmd(lin, ang, "AXIS_ALIGN", False, "align")

    def _enter_blind(self, last_z: float, travel_m: float | None) -> None:
        c = self.cfg
        self.phase = "BLIND_PUSH"
        self._blind_target = max(0.0, last_z - (c.stop_m + c.front_offset_m))
        self._blind_travel0 = travel_m
        self._progress_ref = None

    def _do_blind_push(self, travel_m: float, now_s: float) -> Cmd:
        c = self.cfg
        if self._blind_travel0 is None:
            self._blind_travel0 = travel_m
        if travel_m - self._blind_travel0 >= self._blind_target:
            return self._stop("reached", "DONE")
        if self._progress_ref is None:
            self._progress_ref = (travel_m, now_s)
        else:
            ref_travel, ref_t = self._progress_ref
            if travel_m - ref_travel > 0.005:
                self._progress_ref = (travel_m, now_s)
            elif now_s - ref_t > c.no_progress_s:
                return self._stop("blocked", "ABORT")
        return Cmd(c.lin_pulse, 0.0, "BLIND_PUSH", False, "blind")
```

`step()` 의 마커 처리 부분을 아래로 **교체**한다(타임아웃·`scan_guard` 검사 이후 전체):

```python
        seen = obs is not None and obs.marker_id == c.marker_id
        if seen:
            self._lost = 0
            self._last_z = obs.z_m
            if self.phase in ("SEARCH", "HOMING"):
                self.phase = "AXIS_ALIGN" if obs.z_m <= c.axis_gate_m else "HOMING"
            if self.phase == "HOMING":
                return self._do_homing(obs, dt)
            if self.phase == "AXIS_ALIGN":
                return self._do_axis_align(obs, now_s)
            if self.phase == "BLIND_PUSH":
                return self._do_blind_push(travel_m, now_s)
            return Cmd(0.0, 0.0, self.phase, False, "hold")

        if self.phase == "BLIND_PUSH":
            return self._do_blind_push(travel_m, now_s)
        if self.phase in ("HOMING", "AXIS_ALIGN"):
            self._lost += 1
            if self._lost < c.lost_grace:
                return Cmd(0.0, 0.0, self.phase, False, "lost_grace")
            if self._last_z is not None and self._last_z <= c.lost_near_m:
                self._enter_blind(self._last_z, travel_m)
                return self._do_blind_push(travel_m, now_s)
            self.phase = "SEARCH"
            self._yaw0 = None
            self._target_i = 0
            return Cmd(0.0, 0.0, "SEARCH", False, "lost_far")
        return self._do_search(yaw_deg, now_s)
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `cd /home/ane/personal_repo/arte_aurcomaker_move && python3 -m pytest marker/tests/test_approach.py -v`
Expected: PASS (19 passed)

- [ ] **Step 5: 커밋**

```bash
cd /home/ane/personal_repo/arte_aurcomaker_move
git add marker/approach.py marker/tests/test_approach.py
git commit -m "feat: complete state machine with axis align, blind push and loss branching"
```

---

### Task 4: 검출과 사전 스캔

**Files:**
- Create: `marker/detect.py`
- Test: `marker/tests/test_detect.py`

**Interfaces:**
- Consumes: `MarkerObs` (Task 1)
- Produces:
  - `detect_marker(frame, K, dist, *, marker_len_m: float, target_id: int, dict_name: str) -> MarkerObs | None`
  - `scan_dicts(frame) -> list[tuple[str, list[int]]]` — 프레임에서 검출되는 (사전명, ID 목록)
  - `DICTS: dict[str, int]`
  - `make_marker_image(dict_name: str, marker_id: int, side_px: int)` — 테스트·인쇄용 마커 이미지(4.6/4.7+ 양쪽)

- [ ] **Step 1: 실패하는 테스트 작성**

`marker/tests/test_detect.py`:

```python
import numpy as np
import pytest

from marker.detect import detect_marker, make_marker_image, scan_dicts

W, H = 640, 480
FX = FY = 609.2
K = np.array([[FX, 0, W / 2], [0, FY, H / 2], [0, 0, 1]], dtype=np.float64)
DIST = np.zeros(5, dtype=np.float64)
LEN_M = 0.07
DICT = "DICT_5X5_100"


def render(marker_id=1, z_m=1.0, offset_px=0, dict_name=DICT):
    """카메라 정면 z_m 거리에 마커가 있는 것처럼 합성 프레임을 만든다."""
    side = int(round(LEN_M * FX / z_m))
    img = make_marker_image(dict_name, marker_id, side)
    frame = np.full((H, W), 255, dtype=np.uint8)
    x0 = (W - side) // 2 + offset_px
    y0 = (H - side) // 2
    frame[y0:y0 + side, x0:x0 + side] = img
    return np.dstack([frame] * 3)


def test_none_when_blank():
    blank = np.full((H, W, 3), 255, dtype=np.uint8)
    assert detect_marker(blank, K, DIST, marker_len_m=LEN_M,
                         target_id=1, dict_name=DICT) is None


def test_none_for_other_id():
    assert detect_marker(render(marker_id=7), K, DIST, marker_len_m=LEN_M,
                         target_id=1, dict_name=DICT) is None


@pytest.mark.parametrize("z", [0.5, 1.0, 2.0])
def test_distance_within_five_percent(z):
    o = detect_marker(render(z_m=z), K, DIST, marker_len_m=LEN_M,
                      target_id=1, dict_name=DICT)
    assert o is not None
    assert abs(o.z_m - z) / z < 0.05


def test_ex_sign_follows_offset():
    right = detect_marker(render(offset_px=+80), K, DIST, marker_len_m=LEN_M,
                          target_id=1, dict_name=DICT)
    left = detect_marker(render(offset_px=-80), K, DIST, marker_len_m=LEN_M,
                         target_id=1, dict_name=DICT)
    assert right.ex > 0.1 and left.ex < -0.1


def test_centered_marker_is_square_on():
    o = detect_marker(render(), K, DIST, marker_len_m=LEN_M, target_id=1, dict_name=DICT)
    assert abs(o.yaw_deg) < 5.0
    assert abs(o.lateral_m) < 0.02


def test_wrong_dictionary_finds_nothing():
    """5X5_100 마커를 4X4_50 으로 찾으면 검출 0개 — 조용한 실패의 정체."""
    assert detect_marker(render(), K, DIST, marker_len_m=LEN_M,
                         target_id=1, dict_name="DICT_4X4_50") is None


def test_scan_dicts_identifies_the_right_dictionary():
    hits = dict(scan_dicts(render()))
    assert DICT in hits
    assert 1 in hits[DICT]
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `cd /home/ane/personal_repo/arte_aurcomaker_move && python3 -m pytest marker/tests/test_detect.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'marker.detect'`

- [ ] **Step 3: 구현**

`marker/detect.py`:

```python
"""프레임 한 장 → MarkerObs.

자세 추정은 IPPE_SQUARE 고정이다. aba_project 의 기존 도킹 구현이
"estimatePoseSingleMarkers(ITERATIVE)는 작은 마커에서 yaw 부호가 튀어 축 접근을
망친다"는 이유로 이미 고른 방식이다.

cv2.aruco API 는 4.6 과 4.7+ 가 다르다(ArucoDetector / generateImageMarker 는 4.7+).
로봇과 노트북의 설치 버전이 다를 수 있으므로 양쪽을 지원한다.
"""
import cv2
import numpy as np

from .types import MarkerObs

DICTS = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
}


def _dictionary(dict_name: str):
    if dict_name not in DICTS:
        raise ValueError(f"지원하지 않는 사전: {dict_name} (가능: {', '.join(DICTS)})")
    ident = DICTS[dict_name]
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(ident)
    return cv2.aruco.Dictionary_get(ident)          # 아주 오래된 빌드


def _params():
    p = (cv2.aruco.DetectorParameters() if hasattr(cv2.aruco, "ArucoDetector")
         else cv2.aruco.DetectorParameters_create())
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return p


def _detect_raw(gray, dict_name: str):
    dictionary, params = _dictionary(dict_name), _params()
    if hasattr(cv2.aruco, "ArucoDetector"):                     # 4.7+
        return cv2.aruco.ArucoDetector(dictionary, params).detectMarkers(gray)
    return cv2.aruco.detectMarkers(gray, dictionary, parameters=params)   # 4.6


def make_marker_image(dict_name: str, marker_id: int, side_px: int):
    """마커 이미지(그레이스케일). 테스트와 인쇄용 생성 양쪽에 쓴다."""
    dictionary = _dictionary(dict_name)
    if hasattr(cv2.aruco, "generateImageMarker"):               # 4.7+
        return cv2.aruco.generateImageMarker(dictionary, marker_id, side_px)
    return cv2.aruco.drawMarker(dictionary, marker_id, side_px)  # 4.6


def _gray(frame):
    return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame


def scan_dicts(frame) -> list[tuple[str, list[int]]]:
    """어떤 사전으로 만든 마커인지 모를 때 전부 훑어 알려 준다.

    사전이 틀리면 예외가 아니라 '검출 0개'로 조용히 실패하기 때문에 필요하다.
    """
    gray = _gray(frame)
    found = []
    for name in DICTS:
        _, ids, _ = _detect_raw(gray, name)
        if ids is not None and len(ids):
            found.append((name, sorted(int(i) for i in ids.flatten())))
    return found


def detect_marker(frame, K, dist, *, marker_len_m: float, target_id: int,
                  dict_name: str) -> MarkerObs | None:
    """대상 ID 마커를 찾아 관측값을 만든다. 없으면 None."""
    gray = _gray(frame)
    corners, ids, _ = _detect_raw(gray, dict_name)
    if ids is None:
        return None
    hit = next((c for c, i in zip(corners, ids.flatten()) if int(i) == target_id), None)
    if hit is None:
        return None

    h, w = gray.shape[:2]
    pts = hit.reshape(4, 2).astype(np.float32)
    if min(np.ptp(pts[:, 0]), np.ptp(pts[:, 1])) < 8.0:
        return None                          # 8px 미만은 유령 검출로 버린다
    ex = (float(pts[:, 0].mean()) - w / 2.0) / (w / 2.0)
    side_px = float(np.mean([np.linalg.norm(pts[(i + 1) % 4] - pts[i]) for i in range(4)]))

    s = marker_len_m / 2.0
    obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float32)
    ok, rvec, tvec = cv2.solvePnP(obj, pts, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    tvec = tvec.reshape(3)
    R, _ = cv2.Rodrigues(rvec.reshape(3))
    yaw = float(np.degrees(np.arctan2(-R[2, 0], np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2))))
    lateral = float((R.T @ -tvec)[0])        # 마커 법선축에서 벗어난 거리
    return MarkerObs(marker_id=target_id, ex=ex, z_m=float(tvec[2]), x_m=float(tvec[0]),
                     yaw_deg=yaw, lateral_m=lateral, size_frac=side_px / float(w))
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `cd /home/ane/personal_repo/arte_aurcomaker_move && python3 -m pytest marker/tests/test_detect.py -v`
Expected: PASS (9 passed)

- [ ] **Step 5: 커밋**

```bash
cd /home/ane/personal_repo/arte_aurcomaker_move
git add marker/detect.py marker/tests/test_detect.py
git commit -m "feat: add ArUco detection, pose estimation and dictionary scan (cv2 4.6/4.7+)"
```

---

### Task 5: 캘리브 로더와 카메라 이음매

**Files:**
- Create: `marker/calib.py`
- Create: `marker/camera.py`

**Interfaces:**
- Consumes: 없음
- Produces:
  - `load_calib(slot: str) -> tuple[np.ndarray, np.ndarray]` — (K, dist). 슬롯은 `front`/`back`
  - `CALIB_FILES: dict[str, str]`
  - `open_camera(source: str, *, width: int = 640, height: int = 480, rotate: int = 180) -> Camera`
  - `Camera.get_frame() -> np.ndarray` — ★ 3단계 승격 시 **이 함수만** 바뀐다
  - `Camera.close() -> None`

**주의:** `camera.py` 는 자동 테스트하지 않는다(실제 장치가 본질). `calib.py` 는 Task 1 에서
복사한 실제 파일을 읽어 확인한다.

- [ ] **Step 1: 캘리브 로더 작성과 확인**

`marker/calib.py`:

```python
"""슬롯 이름이 곧 캘리브레이션 선택 키다.

주점(cx)이 회전 여부에 따라 80px 넘게 다르다(rot180: 278.2 vs 비회전: 360.8).
잘못 물리면 거리는 그럴듯한데 정렬만 한쪽으로 흐른다 — 가장 찾기 어려운 버그다.
"""
import pathlib

import numpy as np

CALIB_DIR = pathlib.Path(__file__).resolve().parents[1] / "config" / "camera"
CALIB_FILES = {
    "front": "picam_640x480_rot180.npz",   # picam CSI, rotate 180
    "back": "usb_640x480.npz",             # USB 웹캠 (3단계용)
}


def load_calib(slot: str):
    if slot not in CALIB_FILES:
        raise ValueError(f"슬롯은 front/back 중 하나여야 한다: {slot}")
    path = CALIB_DIR / CALIB_FILES[slot]
    if not path.exists():
        raise FileNotFoundError(f"캘리브레이션이 없다: {path}")
    data = np.load(path)
    return data["camera_matrix"], data["dist_coeffs"]
```

확인:

```bash
cd /home/ane/personal_repo/arte_aurcomaker_move
python3 -c "
from marker.calib import load_calib
K, d = load_calib('front'); assert abs(K[0,0]-609.2) < 1 and abs(K[0,2]-278.2) < 1
K2, _ = load_calib('back'); assert abs(K2[0,0]-675.0) < 1
try:
    load_calib('side')
except ValueError:
    print('OK: front/back 로드, 잘못된 슬롯 거부')
"
```
Expected: `OK: front/back 로드, 잘못된 슬롯 거부`

- [ ] **Step 2: 카메라 이음매 작성**

`marker/camera.py`:

```python
"""프레임 획득 — 이 파일이 이음매다.

1단계(이 레포, 단독 실행)는 장치를 직접 연다. 다른 스택이 안 떠 있으므로 경합이 없다.

3단계에서 aba_project 미션 BT 로 승격하면 그때는 영상 송출 프로세스가 카메라를 잡고
있다. 장치를 직접 열면 앞캠이 'Device or resource busy' 로 죽는다. 그때는 아래
get_frame() 한 곳만 공유메모리 탭 읽기로 바꾸면 되고, 상태기계는 손대지 않는다.

CSI(picam)는 일반 VideoCapture 로 검은 화면만 나오므로 picamera2 를 거친다.
"""
import cv2

_ROTATE = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
           270: cv2.ROTATE_90_COUNTERCLOCKWISE}


class Camera:
    """get_frame() 하나만 노출한다. 어디서 오는지는 호출자가 몰라야 한다."""

    def __init__(self, reader, closer, rotate: int = 0):
        self._read = reader
        self._close = closer
        self._rotate = rotate

    def get_frame(self):
        frame = self._read()
        if frame is None:
            return None
        if self._rotate in _ROTATE:
            frame = cv2.rotate(frame, _ROTATE[self._rotate])
        return frame

    def close(self) -> None:
        self._close()


def _open_csi(width: int, height: int) -> Camera:
    try:
        from picamera2 import Picamera2
    except ImportError:
        raise SystemExit(
            "picamera2 가 없다. CSI 카메라는 시스템 패키지가 필요하다:\n"
            "  sudo apt install -y python3-picamera2\n"
            "그리고 가상환경이 아니라 시스템 python3 로 실행해라.")
    cam = Picamera2()
    cam.configure(cam.create_preview_configuration(
        main={"size": (width, height), "format": "RGB888"}))   # BGR 순서로 나온다
    cam.start()
    return Camera(cam.capture_array, cam.stop)


def _open_usb(index: int, width: int, height: int) -> Camera:
    cap = cv2.VideoCapture(index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not cap.isOpened():
        raise SystemExit(f"카메라를 열 수 없다: index={index}")

    def read():
        ok, frame = cap.read()
        return frame if ok else None

    return Camera(read, cap.release)


def open_camera(source: str = "csi", *, width: int = 640, height: int = 480,
                rotate: int = 180) -> Camera:
    """source: 'csi' 또는 USB 장치 인덱스 문자열('0', '1', ...).

    rotate 기본 180 은 이 Pi 의 CSI 카메라가 거꾸로 장착돼 있기 때문이며,
    config/camera/picam_640x480_rot180.npz 가 그 상태로 캘리브된 것이다.
    """
    cam = _open_csi(width, height) if source == "csi" else _open_usb(int(source), width, height)
    cam._rotate = rotate
    return cam
```

- [ ] **Step 3: 문법·구조 확인 (장치 없이)**

Run:
```bash
cd /home/ane/personal_repo/arte_aurcomaker_move
python3 -c "
import inspect
from marker import camera
src = inspect.getsource(camera)
assert 'def get_frame' in src
# 상태기계에 카메라 호출이 새어 나가지 않았는지 확인
from marker import approach
a = inspect.getsource(approach)
assert 'cv2' not in a and 'camera' not in a and 'rclpy' not in a
print('OK: 이음매 존재, 상태기계는 카메라/ROS 를 모른다')
"
```
Expected: `OK: 이음매 존재, 상태기계는 카메라/ROS 를 모른다`

- [ ] **Step 4: 커밋**

```bash
cd /home/ane/personal_repo/arte_aurcomaker_move
git add marker/calib.py marker/camera.py
git commit -m "feat: add slot-keyed calibration loader and camera acquisition seam"
```

---

### Task 6: ROS 어댑터와 주행 CLI

**Files:**
- Create: `marker/odom.py`, `marker/scan.py`, `marker/drive.py`
- Create: `marker-drive.sh`

**Interfaces:**
- Consumes: Task 1-5 전부
- Produces:
  - `OdomTracker(node)` — `.yaw_deg`(누적, 언랩), `.travel_m`(누적 전진거리)
  - `ScanWatch(node, half_angle_deg=15.0)` — `.front_m`
  - CLI: `python3 -m marker.drive {drive|detect|stop}`

- [ ] **Step 1: ROS 어댑터 작성**

`marker/odom.py`:

```python
"""/odom 에서 누적 방위각과 누적 전진거리를 뽑는다.

시간 기반 회전 추정을 쓰지 않는 이유: 같은 시간을 돌아도 바닥 재질과 배터리 잔량에
따라 회전량이 달라진다. 20° 스텝을 6번 쌓으면 그 오차가 누적돼 스윕이 원점으로
돌아오지 않는다.
"""
import math

from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data


def _yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OdomTracker:
    def __init__(self, node, topic: str = "/odom"):
        self.yaw_deg = 0.0
        self.travel_m = 0.0
        self._prev_yaw = None
        self._prev_xy = None
        node.create_subscription(Odometry, topic, self._on_odom, qos_profile_sensor_data)

    def _on_odom(self, msg) -> None:
        p = msg.pose.pose.position
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        if self._prev_yaw is not None:
            d = yaw - self._prev_yaw
            while d > math.pi:
                d -= 2 * math.pi
            while d < -math.pi:
                d += 2 * math.pi
            self.yaw_deg += math.degrees(d)
        self._prev_yaw = yaw
        if self._prev_xy is not None:
            self.travel_m += math.hypot(p.x - self._prev_xy[0], p.y - self._prev_xy[1])
        self._prev_xy = (p.x, p.y)
```

`marker/scan.py`:

```python
"""원본 /scan 전방 섹터의 최소거리 — 유일한 근접 보호다.

필터된 스캔(scan_filtered)을 쓰지 않는 이유: 그쪽은 min_range 0.05 로 5cm 미만을
제거한다. 근접 감시에는 그 구간이 필요하다.
"""
import math

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanWatch:
    def __init__(self, node, topic: str = "/scan", half_angle_deg: float = 15.0):
        self.front_m = None
        self._half = math.radians(half_angle_deg)
        node.create_subscription(LaserScan, topic, self._on_scan, qos_profile_sensor_data)

    def _on_scan(self, msg) -> None:
        best = None
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue
            a = msg.angle_min + i * msg.angle_increment
            a = math.atan2(math.sin(a), math.cos(a))
            if abs(a) <= self._half and (best is None or r < best):
                best = r
        self.front_m = best
```

- [ ] **Step 2: CLI 작성**

`marker/drive.py`:

```python
#!/usr/bin/env python3
"""마커 주행 CLI — 로봇에서 실행한다.

    python3 -m marker.drive drive              마커 찾아 앞 10cm 까지
    python3 -m marker.drive detect             모터 정지, 검출값만 출력
    python3 -m marker.drive detect --scan-dicts   어떤 사전의 마커인지 훑어본다
    python3 -m marker.drive stop               /cmd_vel 0 발행 후 종료

전제: 로봇 구동 노드(/cmd_vel 구독, /odom·/scan 발행)가 떠 있어야 한다.
      다른 스택(추종·영상송출·nav2)은 떠 있으면 안 된다 — 카메라와 /cmd_vel 을 다툰다.
"""
import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import Twist

from .approach import MarkerApproach
from .calib import load_calib
from .camera import open_camera
from .config import MarkerDriveConfig
from .detect import detect_marker, scan_dicts
from .odom import OdomTracker
from .scan import ScanWatch


def _parse(argv):
    ap = argparse.ArgumentParser(description="ArUco 마커 주행 (nav2 미사용)")
    ap.add_argument("mode", choices=["drive", "detect", "stop"])
    ap.add_argument("--source", default="csi", help="'csi' 또는 USB 인덱스('0')")
    ap.add_argument("--slot", default="front", choices=["front", "back"],
                    help="캘리브 선택 키")
    ap.add_argument("--rotate", type=int, default=180, choices=[0, 90, 180, 270])
    ap.add_argument("--scan-dicts", dest="scan_dicts", action="store_true",
                    help="detect 모드에서 어떤 사전의 마커인지 훑어본다")
    ap.add_argument("--marker-id", type=int, default=1)
    ap.add_argument("--marker-m", type=float, default=0.07)
    ap.add_argument("--dict", dest="dict_name", default="DICT_5X5_100")
    ap.add_argument("--stop-m", type=float, default=0.10, help="마커~로봇 최전방 목표")
    ap.add_argument("--front-offset", type=float, default=0.0, help="카메라~로봇 최전방")
    ap.add_argument("--steer-sign", type=float, default=1.0, help="반대로 돌면 -1")
    ap.add_argument("--axis-gate", type=float, default=0.6, help="yaw 신뢰 시작 거리")
    ap.add_argument("--lin-homing", type=float, default=0.12)
    ap.add_argument("--lin-pulse", type=float, default=0.08)
    ap.add_argument("--ang-search", type=float, default=0.35)
    ap.add_argument("--search-step-deg", type=float, default=20.0)
    ap.add_argument("--search-span-deg", type=float, default=60.0)
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--loop-hz", type=float, default=12.0)
    ap.add_argument("--cmd-topic", default="/cmd_vel")
    return ap.parse_args(argv)


def _config(a) -> MarkerDriveConfig:
    return MarkerDriveConfig(
        marker_id=a.marker_id, marker_len_m=a.marker_m, dict_name=a.dict_name,
        stop_m=a.stop_m, front_offset_m=a.front_offset, steer_sign=a.steer_sign,
        axis_gate_m=a.axis_gate, lin_homing=a.lin_homing, lin_pulse=a.lin_pulse,
        ang_search=a.ang_search, search_step_deg=a.search_step_deg,
        search_span_deg=a.search_span_deg, timeout_s=a.timeout, loop_hz=a.loop_hz,
    ).clamped()


def _publish(pub, lin: float, ang: float) -> None:
    t = Twist()
    t.linear.x = float(lin)
    t.angular.z = float(ang)
    pub.publish(t)


def main(argv=None) -> int:
    a = _parse(argv if argv is not None else sys.argv[1:])
    cfg = _config(a)
    rclpy.init()
    node = rclpy.create_node("marker_drive")
    pub = node.create_publisher(Twist, a.cmd_topic, 10)

    if a.mode == "stop":
        for _ in range(5):
            _publish(pub, 0.0, 0.0)
            time.sleep(0.05)
        node.destroy_node()
        rclpy.shutdown()
        print("[ok] 정지 명령 발행 완료")
        return 0

    K, dist = load_calib(a.slot)
    cam = open_camera(a.source, rotate=a.rotate)
    odom = OdomTracker(node)
    watch = ScanWatch(node)
    machine = MarkerApproach(cfg)
    period = 1.0 / cfg.loop_hz
    print(f"[ok] mode={a.mode} source={a.source} slot={a.slot} id={cfg.marker_id} "
          f"marker={cfg.marker_len_m}m stop={cfg.stop_m}m(+{cfg.front_offset_m}) "
          f"gate={cfg.axis_gate_m}m sign={cfg.steer_sign:+.0f} dict={cfg.dict_name}")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            frame = cam.get_frame()
            if frame is None:
                _publish(pub, 0.0, 0.0)
                print("[error] 카메라 프레임이 없다 — 장치를 확인해라")
                return 2

            if a.mode == "detect":
                if a.scan_dicts:
                    hits = scan_dicts(frame)
                    print("검출된 사전: " + (", ".join(f"{n}{ids}" for n, ids in hits)
                                             if hits else "없음"))
                else:
                    o = detect_marker(frame, K, dist, marker_len_m=cfg.marker_len_m,
                                      target_id=cfg.marker_id, dict_name=cfg.dict_name)
                    front = f"{watch.front_m:.3f}m" if watch.front_m else "--"
                    if o is None:
                        print(f"marker: --   scan_front={front}")
                    else:
                        remain = o.z_m - (cfg.stop_m + cfg.front_offset_m)
                        print(f"marker: z={o.z_m:.3f}m ex={o.ex:+.3f} yaw={o.yaw_deg:+.1f} "
                              f"lat={o.lateral_m:+.3f} size={o.size_frac:.2f} | "
                              f"앞면까지 남음 {remain:+.3f}m  scan_front={front}")
                time.sleep(max(period, 0.5))
                continue

            o = detect_marker(frame, K, dist, marker_len_m=cfg.marker_len_m,
                              target_id=cfg.marker_id, dict_name=cfg.dict_name)
            cmd = machine.step(o, yaw_deg=odom.yaw_deg, travel_m=odom.travel_m,
                               front_m=watch.front_m, now_s=time.monotonic())
            _publish(pub, cmd.linear, cmd.angular)
            print(f"[{cmd.phase:<10}] lin={cmd.linear:+.3f} ang={cmd.angular:+.3f} "
                  f"reason={cmd.reason}")
            if cmd.done:
                _publish(pub, 0.0, 0.0)
                ok = cmd.phase == "DONE"
                print("[ok] 도착" if ok else f"[fail] 중단: {cmd.reason}")
                return 0 if ok else 1
            time.sleep(period)
        return 0
    except KeyboardInterrupt:
        print("\n[stop] 사용자 중단 — 모터 정지")
        return 130
    finally:
        for _ in range(3):
            _publish(pub, 0.0, 0.0)
        cam.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 3: 런처 작성**

`marker-drive.sh`:

```bash
#!/usr/bin/env bash
# 로봇에서 실행 — ArUco 마커 한 장을 보고 마커 앞 10cm 까지 주행한다(nav2 미사용).
#
#   ./marker-drive.sh                       마커 ID 1 을 찾아 앞 10cm 까지
#   ./marker-drive.sh detect                모터 정지 상태로 검출값만(튜닝용)
#   ./marker-drive.sh detect --scan-dicts   벽 마커가 어떤 사전인지 확인
#   ./marker-drive.sh stop                  즉시 정지 — 모터를 물고 있으므로 비상구다
#   ./marker-drive.sh drive --steer-sign -1 --axis-gate 0.5     플래그는 그대로 위임
#
# ⚠️ 전제 1: 로봇 구동 노드가 떠 있어야 한다(/cmd_vel 구독, /odom·/scan 발행).
# ⚠️ 전제 2: 다른 스택(추종·영상송출·nav2)은 떠 있으면 안 된다.
#            카메라 장치와 /cmd_vel 을 다툰다.
# ⚠️ 시스템 python3 로 돈다 — picamera2 는 시스템 패키지라 venv 에서 안 잡힌다.
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

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

ARGS=("$MODE")
[ -n "${SOURCE:-}" ]       && ARGS+=(--source "$SOURCE")
[ -n "${SLOT:-}" ]         && ARGS+=(--slot "$SLOT")
[ -n "${MARKER_ID:-}" ]    && ARGS+=(--marker-id "$MARKER_ID")
[ -n "${MARKER_M:-}" ]     && ARGS+=(--marker-m "$MARKER_M")
[ -n "${DICT:-}" ]         && ARGS+=(--dict "$DICT")
[ -n "${STOP_M:-}" ]       && ARGS+=(--stop-m "$STOP_M")
[ -n "${FRONT_OFFSET:-}" ] && ARGS+=(--front-offset "$FRONT_OFFSET")
[ -n "${STEER_SIGN:-}" ]   && ARGS+=(--steer-sign "$STEER_SIGN")
[ -n "${AXIS_GATE:-}" ]    && ARGS+=(--axis-gate "$AXIS_GATE")
ARGS+=("$@")

cd "$HERE"
exec python3 -m marker.drive "${ARGS[@]}"
```

```bash
chmod +x /home/ane/personal_repo/arte_aurcomaker_move/marker-drive.sh
```

- [ ] **Step 4: 인자 전달 검증 (ROS·장치 없이)**

Run:
```bash
cd /home/ane/personal_repo/arte_aurcomaker_move
python3 -c "
import sys, types
# rclpy 계열이 없어도 파서만 확인할 수 있게 최소 스텁을 끼운다
for name in ['rclpy', 'rclpy.qos', 'geometry_msgs', 'geometry_msgs.msg',
             'nav_msgs', 'nav_msgs.msg', 'sensor_msgs', 'sensor_msgs.msg']:
    sys.modules.setdefault(name, types.ModuleType(name))
sys.modules['geometry_msgs.msg'].Twist = object
sys.modules['nav_msgs.msg'].Odometry = object
sys.modules['sensor_msgs.msg'].LaserScan = object
sys.modules['rclpy.qos'].qos_profile_sensor_data = None
from marker.drive import _parse, _config
c = _config(_parse(['drive','--steer-sign','-1','--axis-gate','0.45','--stop-m','0.12']))
assert (c.steer_sign, c.axis_gate_m, c.stop_m) == (-1.0, 0.45, 0.12)
a = _parse(['detect','--scan-dicts']); assert a.scan_dicts and a.mode == 'detect'
print('OK: 플래그가 설정으로 전달된다')
"
bash -n marker-drive.sh && echo "OK: 런처 문법"
```
Expected: `OK: 플래그가 설정으로 전달된다` 와 `OK: 런처 문법`

- [ ] **Step 5: 커밋**

```bash
cd /home/ane/personal_repo/arte_aurcomaker_move
git add marker/odom.py marker/scan.py marker/drive.py marker-drive.sh
git commit -m "feat: add odom/scan adapters and marker drive CLI with dict scan"
```

---

### Task 7: 노트북 관찰 도구와 README

**Files:**
- Create: `marker/watch.py`, `marker-watch.sh`, `README.md`

**Interfaces:**
- Consumes: Task 1-6 전부
- Produces: 노트북에서 도는 검출 오버레이(모터 무접촉), 레포 사용 설명서

- [ ] **Step 1: 관찰 도구 작성**

`marker/watch.py`:

```python
#!/usr/bin/env python3
"""노트북에서 카메라 영상에 마커 검출 오버레이를 씌워 본다. 모터를 건드리지 않는다.

용도: 마커가 몇 m 부터 잡히는지, yaw 가 어느 거리부터 안정되는지 보고
      로봇 쪽 --axis-gate 값을 정하는 것.

노트북에 붙은 USB 캠으로 직접 볼 수도 있고(--source 0),
로봇에서 같은 마커를 같은 조건으로 확인할 때는 로봇에서 --source csi 로 돌린다.
"""
import argparse

import cv2

from .calib import load_calib
from .camera import open_camera
from .detect import detect_marker, scan_dicts


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="마커 검출 관찰(모터 무접촉)")
    ap.add_argument("--source", default="0", help="'csi' 또는 USB 인덱스")
    ap.add_argument("--slot", default="back", choices=["front", "back"])
    ap.add_argument("--rotate", type=int, default=0, choices=[0, 90, 180, 270])
    ap.add_argument("--marker-id", type=int, default=1)
    ap.add_argument("--marker-m", type=float, default=0.07)
    ap.add_argument("--dict", dest="dict_name", default="DICT_5X5_100")
    ap.add_argument("--stop-m", type=float, default=0.10)
    ap.add_argument("--scan-dicts", dest="scan_dicts", action="store_true")
    ap.add_argument("--no-window", action="store_true", help="창 없이 텍스트만")
    a = ap.parse_args(argv)

    K, dist = load_calib(a.slot)
    cam = open_camera(a.source, rotate=a.rotate)
    print(f"[ok] source={a.source} slot={a.slot} id={a.marker_id} dict={a.dict_name}")
    try:
        while True:
            frame = cam.get_frame()
            if frame is None:
                print("[error] 프레임 없음")
                return 2
            if a.scan_dicts:
                hits = scan_dicts(frame)
                text = "사전: " + (", ".join(f"{n}{ids}" for n, ids in hits) if hits else "없음")
                obs = None
            else:
                obs = detect_marker(frame, K, dist, marker_len_m=a.marker_m,
                                    target_id=a.marker_id, dict_name=a.dict_name)
                text = ("marker: --" if obs is None else
                        f"z={obs.z_m:.3f}m ex={obs.ex:+.3f} yaw={obs.yaw_deg:+.1f} "
                        f"lat={obs.lateral_m:+.3f} 남음={obs.z_m - a.stop_m:+.3f}m")
            print(text, flush=True)
            if not a.no_window:
                cv2.putText(frame, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 255, 0) if obs else (0, 0, 255), 2)
                cv2.imshow("marker-watch", frame)
                if cv2.waitKey(30) & 0xFF == ord("q"):
                    break
            else:
                cv2.waitKey(1)
    except KeyboardInterrupt:
        pass
    finally:
        cam.close()
        if not a.no_window:
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

`marker-watch.sh`:

```bash
#!/usr/bin/env bash
# 노트북(또는 로봇)에서 실행 — 마커 검출 오버레이. 모터를 건드리지 않는다.
#
#   ./marker-watch.sh                    노트북 USB 캠(index 0)으로 관찰
#   ./marker-watch.sh --scan-dicts       벽 마커가 어떤 사전인지 확인
#   ./marker-watch.sh --source csi --slot front --rotate 180    로봇 CSI 캠으로
#   ./marker-watch.sh --no-window        화면 없는 환경
#
# 용도: 마커가 몇 m 부터 잡히는지, yaw 가 어느 거리부터 안정되는지 보고
#       주행 쪽 --axis-gate 값을 정한다. ROS 가 필요 없다.
set -eo pipefail
cd "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python3 -m marker.watch "$@"
```

```bash
chmod +x /home/ane/personal_repo/arte_aurcomaker_move/marker-watch.sh
```

- [ ] **Step 2: README 작성**

`README.md`:

```markdown
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

## 동작

| 단계 | 하는 일 | 다음으로 |
|---|---|---|
| `SEARCH` | 좌우 ±60° 를 20° 씩 끊어 돌며 멈춰서 본다. 회전량은 `/odom` 으로 닫는다 | 검출 → `HOMING` / 1스윕 실패 → 중단 |
| `HOMING` | 화면 중앙 오차 PID 로 조향하며 전진. **yaw 는 안 쓴다**(먼 거리에서 뒤집힌다) | `z ≤ 0.6m` → `AXIS_ALIGN` |
| `AXIS_ALIGN` | yaw·축이탈로 마커 법선축에 올라탄다. 펄스 이동 | `z ≤ 0.10m` → 완료 / `z ≤ 0.25m` → `BLIND_PUSH` |
| `BLIND_PUSH` | 마커가 시야를 벗어난 마지막 2~3cm 를 `/odom` 으로 간다 | 도달 → 완료 |

7cm 마커는 약 12.6cm 부터 화면에서 잘리기 시작한다. 그래서 마지막 구간만 눈을 감는다.

## 실기에서만 정해지는 값

| 플래그 | 기본 | 정하는 법 |
|---|---|---|
| `--steer-sign` | `+1` | 로봇이 반대로 흐르면 `-1`. 소프트웨어로 판단 불가 |
| `--axis-gate` | `0.6` | `marker-watch.sh` 로 yaw 가 안정되는 거리를 보고 |
| `--front-offset` | `0.0` | `detect` 의 "앞면까지 남음" 과 자로 잰 값 비교 |
| `--dict` | `DICT_5X5_100` | `--scan-dicts` 로 실물 확정 |

env 로도 준다: `STEER_SIGN=-1 ./marker-drive.sh`

## 구조

| 파일 | 책임 |
|---|---|
| `marker/approach.py` | **판단 전부.** 순수 상태기계 — 카메라·모터·시계·ROS 를 모른다 |
| `marker/detect.py` | 프레임 → 관측값. IPPE_SQUARE 고정, cv2 4.6/4.7+ 양쪽 지원 |
| `marker/camera.py` | **프레임 획득 이음매.** 다른 스택 위로 올릴 때 여기만 바꾼다 |
| `marker/calib.py` | 슬롯(front/back) → 캘리브 파일 |
| `marker/odom.py` `scan.py` | 누적 yaw·전진거리, 원본 `/scan` 전방 최소거리 |
| `marker/drive.py` `watch.py` | CLI 배선 / 관찰 |

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
```

- [ ] **Step 3: 전체 테스트 실행**

Run:
```bash
cd /home/ane/personal_repo/arte_aurcomaker_move
python3 -m pytest marker/tests/ -v
```
Expected: PASS — `test_config.py` 4 + `test_approach.py` 19 + `test_detect.py` 9 = **32 passed**

- [ ] **Step 4: 커밋**

```bash
cd /home/ane/personal_repo/arte_aurcomaker_move
git add marker/watch.py marker-watch.sh README.md
git commit -m "feat: add detection watcher and repo README"
```

---

## 요구사항 커버리지 자체점검

PRD User Story → Task 대조.

| Story | Task | 근거 |
|---|---|---|
| 1, 2 (10cm·최전방 기준) | 3, 6 | `stop_m + front_offset_m` 판정, `test_reaching_stop_distance_while_visible_is_done` |
| 3, 4, 5 (탐색·20° 스텝·1스윕 후 중단) | 2 | `_sweep_targets`, `test_search_aborts_after_full_sweep` |
| 6 (Ctrl-C 즉시 정지) | 6 | `KeyboardInterrupt` → 0 발행 |
| 7 (비스듬 도착 방지) | 3 | `_do_axis_align`, `align_stall` |
| 8 (벽 박기 방지) | 2, 6 | `scan_guard`, `ScanWatch` |
| 9 (단독 실행) | 전체 | Global Constraints, 외부 임포트 없음 |
| 10, 11, 12 (극성·게이트·오프셋 플래그) | 6 | `--steer-sign`, `--axis-gate`, `--front-offset` |
| 13, 14 (검출 모드·남은 거리) | 6 | `detect` 출력 |
| 15 (사전 자동 판별) | 4, 6, 7 | `scan_dicts`, `--scan-dicts` |
| 16 (빌드 없이 튜닝) | 전체 | 순수 파이썬, colcon 없음 |
| 17, 18, 19 (노트북 관찰·모터 무접촉) | 7 | `watch.py` |
| 20 (순수 상태기계) | 2, 3 | `approach.py` |
| 21, 22 (이음매·상태기계 격리) | 5 | `camera.get_frame`, Task 5 Step 3 검증 |
| 23 (후진 확장) | 1, 5 | `slot` 파라미터, `back` 캘리브 포함 |
| 24, 25 (검출 방식·상수 전사) | 1, 4 | IPPE_SQUARE, `config.py` |
| 26 (cv2 4.6/4.7+) | 4 | `_detect_raw`, `make_marker_image` 분기 |
| 27 (CSI/USB 분기 한 곳) | 5 | `open_camera` |
| 28, 29 (슬롯=K 선택, 레포 내 캘리브) | 1, 5 | `calib.py`, Task 1 복사 |
| 30 (카메라 죽음 구분) | 6 | `frame is None` → 종료코드 2 |
| 31, 32 (odom 회전·전진) | 3, 6 | `OdomTracker` |
| 33 (sudo 없음) | 6 | `/cmd_vel` 발행만 |
| 34 (상실 분기) | 3 | `lost_near_m`, 테스트 2건 |
| 35, 36 (미수렴·무진전 중단) | 3 | `align_stall`, `blocked` |
| 37 (원본 scan) | 6 | `scan.py` 주석·구현 |
| 38 (실행 전 확인) | 6, 7 | 런처 가드 + README |
| 39 (하드웨어 없는 테스트) | 1-4 | 32개 전부 하드웨어 불요 |
| 40 (외부 임포트 없음) | 전체 | Global Constraints |

`covered` 100%, `missing` 없음.

## 이전 계획과의 차이 (aba_project 기준 → 이 레포 기준)

| 항목 | 이전 | 지금 |
|---|---|---|
| 위치 | `aba_project/scripts/drive-pi/marker/` | 단독 레포 `arte_aurcomaker_move/marker/` |
| 프레임 획득 | `camera_sender` 생프레임 탭(`/dev/shm`) | **장치 직접 open** + `get_frame()` 이음매 |
| 타 서비스 수정 | `camera_sender.py` 수정 필요 | **없음** |
| `cmd_bridge` 충돌 | `follow-drive` 창 종료 필요 | 스택 자체를 안 띄우므로 무관 |
| 주행목표 취소 결함 의존 | 실기 검증 전제조건 | 무관(nav2 미기동) |
| Task 수 | 10 | **7** |
| 사전 확정 | 가정 | `--scan-dicts` 로 실측 |
