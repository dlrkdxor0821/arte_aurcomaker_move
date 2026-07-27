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
