"""마커 주행 판단 로직 — 순수 상태기계.

카메라·모터·시계·ROS 를 모른다. 입력은 관측값과 센서 수치, 출력은 명령뿐이다.
그래서 하드웨어 없이 전체 판단을 테스트할 수 있고, 나중에 다른 스택 위로 승격할 때
이 파일은 손대지 않는다.

## 단계

    SEARCH ─검출→ HOMING ─z≤gate→ AXIS_ALIGN ─정렬+근접→ BLIND_PUSH ─거리도달→ DONE
       └─1스윕 실패→ ABORT           └─상실(먼 거리)→ SEARCH

## 좌표 약속

- `angular` 양수 = 좌회전(ROS 관례, z 축 CCW).
- `MarkerObs.lateral_m` 는 **마커 로컬 X 축(벽면 수평축) 상의 부호 있는 이탈**이다.
  벽에 붙은 마커 + 지상 주행이므로 수직 성분은 제어에 쓰지 않는다.
- 실제 좌우 극성은 로봇 배선·카메라 장착에 따라 뒤집힐 수 있다. 그건 `steer_sign`
  하나로만 뒤집는다(현장에서 정하는 값).
"""
import math

from .config import MarkerDriveConfig
from .types import Cmd, MarkerObs


def _finite(*values) -> bool:
    """NaN/inf 를 걸러낸다.

    NaN 은 비교가 전부 False 라 조용히 안전장치를 통과한다.
    예: NaN < scan_guard_m 는 False 라서 '안전하다'로 읽힌다.
    """
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)


def _valid_obs(obs: MarkerObs | None) -> bool:
    return obs is not None and _finite(obs.ex, obs.z_m, obs.yaw_deg, obs.lateral_m)


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
        # --- 탐색 ---
        self._targets = _sweep_targets(self.cfg.search_step_deg, self.cfg.search_span_deg)
        self._target_i = 0
        self._yaw0 = None            # 탐색 기준이 되는 시작 방위각
        self._settle_until = 0.0     # 회전 후 재검출 대기 종료 시각
        self._step_started = None    # 현재 스텝 회전 시작 시각(회전 실패 감시)
        # --- 조향 ---
        self._ex_f = None            # ex 저역통과 상태
        self._i_acc = 0.0            # 조향 적분 누적
        self._prev_ex = 0.0
        # --- 진행 상태 ---
        self._t0 = None              # 전체 시작 시각
        self._lost = 0               # 연속 상실 프레임 수
        self._last_z = None          # 마지막으로 본 마커 거리
        self._pulse_until = 0.0      # 펄스 전진 종료 시각
        self._pulse_next = 0.0       # 다음 펄스 시작 가능 시각
        self._blind_target = None    # 무시각 전진 목표 거리(m)
        self._blind_pos0 = None      # 무시각 전진 시작 위치(x, y)
        self._progress_ref = None    # (진행거리, 시각) — 무진전 판정 기준
        self._align_since = None     # 축 정렬 시작 시각

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

    def _aligned(self, o: MarkerObs) -> bool:
        """축 정렬 완료 판정 — lateral 과 yaw 를 **둘 다** 본다.

        lateral 만 보면 yaw 가 크게 틀어진 채로 무시각 구간에 진입해서,
        남은 거리를 비스듬히 밀고 들어간다.
        """
        c = self.cfg
        return abs(o.lateral_m) <= c.pose_axis_tol_m and abs(o.yaw_deg) <= c.pose_yaw_tol_deg

    # ---- 단계 ------------------------------------------------------------
    def _do_search(self, yaw_deg: float, now_s: float) -> Cmd:
        c = self.cfg
        if self._yaw0 is None:
            self._yaw0 = yaw_deg
        if self._target_i >= len(self._targets):
            return self._stop("not_found", "ABORT")
        if now_s < self._settle_until:
            return Cmd(0.0, 0.0, "SEARCH", False, "settle")
        if self._step_started is None:
            self._step_started = now_s
        err = self._targets[self._target_i] - (yaw_deg - self._yaw0)
        if abs(err) <= c.search_tol_deg:          # 도달 → 멈춰서 본다
            self._target_i += 1
            self._settle_until = now_s + c.turn_pause_s
            self._step_started = None
            return Cmd(0.0, 0.0, "SEARCH", False, "look")
        if now_s - self._step_started > c.search_step_timeout_s:
            # 회전 명령을 냈는데 각도가 안 변한다 = 바퀴 헛돎이거나 odom 정지.
            # 전역 타임아웃까지 헛돌게 두면 원인이 'timeout' 으로 오분류된다.
            return self._stop("turn_stall", "ABORT")
        return Cmd(0.0, c.ang_search if err > 0 else -c.ang_search,
                   "SEARCH", False, "sweep")

    def _do_homing(self, o: MarkerObs, dt: float) -> Cmd:
        return Cmd(self.cfg.lin_homing, self._steer(o.ex, dt), "HOMING", False, "approach")

    def _do_axis_align(self, o: MarkerObs, now_s: float, pos_xy) -> Cmd:
        c = self.cfg
        if self._align_since is None:
            self._align_since = now_s
        if o.z_m <= c.stop_m + c.front_offset_m:
            return self._stop("reached", "DONE")
        aligned = self._aligned(o)
        if not aligned and now_s - self._align_since > c.align_stall_s:
            return self._stop("align_stall", "ABORT")
        # heading(yaw) 과 lateral 을 함께 줄여 마커 법선축에 올라탄다
        u = c.pose_kp_yaw * (o.yaw_deg / 90.0) + c.pose_kp_lat * o.lateral_m
        ang = -c.steer_sign * max(-c.steer_ang_max, min(c.steer_ang_max, u))
        if aligned and o.z_m <= c.lost_near_m:
            self._enter_blind(o.z_m, pos_xy)
            return Cmd(0.0, 0.0, "BLIND_PUSH", False, "near")
        if now_s >= self._pulse_next:
            self._pulse_until = now_s + c.move_pulse_s
            self._pulse_next = now_s + c.move_pulse_s + c.move_pause_s
        lin = c.lin_pulse if now_s < self._pulse_until else 0.0
        return Cmd(lin, ang, "AXIS_ALIGN", False, "align")

    def _enter_blind(self, last_z: float, pos_xy) -> None:
        c = self.cfg
        self.phase = "BLIND_PUSH"
        self._blind_target = max(0.0, last_z - (c.stop_m + c.front_offset_m))
        self._blind_pos0 = pos_xy
        self._progress_ref = None

    def _do_blind_push(self, pos_xy, now_s: float) -> Cmd:
        """마커가 시야를 벗어난 마지막 구간을 odom 으로 간다.

        누적 주행거리가 아니라 **시작점에서의 직선 변위**를 쓴다. 누적거리는
        제자리 진동이나 회전으로도 늘어나서, 앞으로 안 갔는데 도달로 읽힌다.
        """
        c = self.cfg
        if self._blind_pos0 is None:
            self._blind_pos0 = pos_xy
        gone = math.hypot(pos_xy[0] - self._blind_pos0[0], pos_xy[1] - self._blind_pos0[1])
        if gone >= self._blind_target:
            return self._stop("reached", "DONE")
        if self._progress_ref is None:
            self._progress_ref = (gone, now_s)
        else:
            ref_gone, ref_t = self._progress_ref
            if gone - ref_gone > 0.005:
                self._progress_ref = (gone, now_s)
            elif now_s - ref_t > c.no_progress_s:
                return self._stop("blocked", "ABORT")
        return Cmd(c.lin_pulse, 0.0, "BLIND_PUSH", False, "blind")

    # ---- 진입점 ----------------------------------------------------------
    def step(self, obs: MarkerObs | None, *, yaw_deg: float, travel_m: float,
             pos_xy: tuple[float, float], front_m: float | None, now_s: float) -> Cmd:
        """한 틱의 판단.

        yaw_deg : odom 누적 방위각(도, 좌회전 +)
        travel_m: odom 누적 주행거리(m) — 진행 감시용
        pos_xy  : odom 좌표 (x, y) — 무시각 구간의 직선 변위 계산용
        front_m : 원본 /scan 전방 최소거리(m). 아직 스캔이 없으면 None
        now_s   : 단조 증가 초
        """
        c = self.cfg
        if self._t0 is None:
            self._t0 = now_s
        if self.phase in ("DONE", "ABORT"):
            return Cmd(0.0, 0.0, self.phase, True, "finished")
        if not _finite(now_s, yaw_deg, travel_m, pos_xy[0], pos_xy[1]):
            return self._stop("bad_odom", "ABORT")
        if now_s - self._t0 > c.timeout_s:
            return self._stop("timeout", "ABORT")
        if front_m is not None:
            if not _finite(front_m):
                front_m = None                    # 값이 깨졌으면 '없음'으로 취급
            elif front_m < c.scan_guard_m:
                return self._stop("scan_guard", "ABORT")
        dt = 1.0 / c.loop_hz

        seen = _valid_obs(obs) and obs.marker_id == c.marker_id
        if seen:
            self._lost = 0
            self._last_z = obs.z_m
            if self.phase in ("SEARCH", "HOMING"):
                self.phase = "AXIS_ALIGN" if obs.z_m <= c.axis_gate_m else "HOMING"
            if self.phase == "HOMING":
                return self._do_homing(obs, dt)
            if self.phase == "AXIS_ALIGN":
                return self._do_axis_align(obs, now_s, pos_xy)
            if self.phase == "BLIND_PUSH":
                return self._do_blind_push(pos_xy, now_s)
            return Cmd(0.0, 0.0, self.phase, False, "hold")

        if self.phase == "BLIND_PUSH":
            return self._do_blind_push(pos_xy, now_s)
        if self.phase in ("HOMING", "AXIS_ALIGN"):
            self._lost += 1
            if self._lost < c.lost_grace:
                return Cmd(0.0, 0.0, self.phase, False, "lost_grace")
            if self._last_z is not None and self._last_z <= c.lost_near_m:
                self._enter_blind(self._last_z, pos_xy)
                return self._do_blind_push(pos_xy, now_s)
            self.phase = "SEARCH"
            self._yaw0 = None
            self._target_i = 0
            self._step_started = None
            return Cmd(0.0, 0.0, "SEARCH", False, "lost_far")
        return self._do_search(yaw_deg, now_s)
