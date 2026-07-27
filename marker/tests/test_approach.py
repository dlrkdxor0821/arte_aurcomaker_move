"""상태기계 테스트 — 하드웨어 없이 전 단계 전이와 중단 조건을 검증한다."""
import pytest

from marker.approach import MarkerApproach
from marker.config import MarkerDriveConfig
from marker.types import MarkerObs


def obs(z, ex=0.0, yaw=0.0, lat=0.0, marker_id=1):
    return MarkerObs(marker_id=marker_id, ex=ex, z_m=z, x_m=0.0,
                     yaw_deg=yaw, lateral_m=lat, size_frac=0.1)


def step(m, o=None, *, yaw=0.0, fwd=0.0, front=None, t=0.0):
    """긴 키워드를 매번 쓰지 않기 위한 얇은 래퍼. fwd = 전진 누적값(m)."""
    return m.step(o, yaw_deg=yaw, forward_m=fwd, front_m=front, now_s=t)


# ---------------------------------------------------------------- 탐색

def test_search_turns_left_first():
    m = MarkerApproach(MarkerDriveConfig())
    c = step(m)
    assert c.phase == "SEARCH"
    assert c.angular > 0
    assert c.linear == 0.0


def test_search_stops_turning_at_step_target():
    m = MarkerApproach(MarkerDriveConfig(search_step_deg=20.0, turn_pause_s=0.4))
    step(m, t=0.0)
    c = step(m, yaw=20.0, t=0.5)
    assert c.angular == 0.0        # 목표 각 도달 → 멈춰서 본다(모션블러 회피)


def test_search_aborts_after_full_sweep():
    m = MarkerApproach(MarkerDriveConfig(search_step_deg=20.0, search_span_deg=60.0,
                                         turn_pause_s=0.0))
    yaw, t, c = 0.0, 0.0, None
    for _ in range(500):
        c = step(m, yaw=yaw, t=t)
        if c.done:
            break
        yaw += 5.0 if c.angular > 0 else (-5.0 if c.angular < 0 else 0.0)
        t += 0.1
    assert c.phase == "ABORT"
    assert c.reason == "not_found"


def test_search_completes_sweep_even_with_coarse_yaw_steps():
    """루프가 느려 한 틱에 여러 도씩 돌아도 스윕이 진행돼야 한다.

    ±tol 창만 보면 5Hz(4°/tick)에서 창을 건너뛰어 도달 판정이 영영 안 난다.
    목표 각 교차로도 판정하므로 통과해야 한다.
    """
    m = MarkerApproach(MarkerDriveConfig(search_step_deg=20.0, search_span_deg=60.0,
                                         search_tol_deg=2.0, turn_pause_s=0.0,
                                         search_step_timeout_s=5.0, timeout_s=600.0))
    yaw, t, c = 0.0, 0.0, None
    for _ in range(400):
        c = step(m, yaw=yaw, t=t)
        if c.done:
            break
        yaw += 12.0 if c.angular > 0 else (-12.0 if c.angular < 0 else 0.0)
        t += 0.2
    assert c.phase == "ABORT" and c.reason == "not_found"   # turn_stall 이 아니어야 한다


def test_search_aborts_when_rotation_makes_no_progress():
    """회전 명령을 내는데 yaw 가 안 변한다 = 바퀴 헛돎이거나 odom 정지.

    이 감시가 없으면 전역 타임아웃까지 헛돌고 원인이 'timeout' 으로 오분류된다.
    """
    m = MarkerApproach(MarkerDriveConfig(search_step_timeout_s=1.0, timeout_s=600.0))
    c = None
    for i in range(50):
        c = step(m, yaw=0.0, t=0.1 * i)       # yaw 가 영영 안 변한다
        if c.done:
            break
    assert c.phase == "ABORT"
    assert c.reason == "turn_stall"


# ---------------------------------------------------------------- 원거리 접근

def test_marker_found_switches_to_homing():
    m = MarkerApproach(MarkerDriveConfig())
    c = step(m, obs(1.5))
    assert c.phase == "HOMING"
    assert c.linear > 0


def test_other_marker_id_is_ignored():
    m = MarkerApproach(MarkerDriveConfig(marker_id=1))
    c = step(m, obs(1.5, marker_id=7))
    assert c.phase == "SEARCH"


def test_homing_steers_proportionally_to_ex():
    small = step(MarkerApproach(MarkerDriveConfig()), obs(1.5, ex=0.10))
    big = step(MarkerApproach(MarkerDriveConfig()), obs(1.5, ex=0.40))
    assert abs(big.angular) > abs(small.angular)


def test_homing_ignores_flipped_yaw_beyond_gate():
    """게이트 밖에서는 yaw 해가 뒤집혀도 명령이 달라지면 안 된다."""
    a = step(MarkerApproach(MarkerDriveConfig()), obs(1.5, ex=0.2, yaw=+170.0))
    b = step(MarkerApproach(MarkerDriveConfig()), obs(1.5, ex=0.2, yaw=-170.0))
    assert a.angular == pytest.approx(b.angular)


def test_homing_respects_deadband():
    m = MarkerApproach(MarkerDriveConfig(steer_deadband=0.05))
    c = step(m, obs(1.5, ex=0.01))
    assert c.angular == 0.0


def test_steer_sign_flips_direction():
    plus = step(MarkerApproach(MarkerDriveConfig(steer_sign=1.0)), obs(1.5, ex=0.3))
    minus = step(MarkerApproach(MarkerDriveConfig(steer_sign=-1.0)), obs(1.5, ex=0.3))
    assert plus.angular == pytest.approx(-minus.angular)


def test_gate_switches_to_axis_align():
    m = MarkerApproach(MarkerDriveConfig(axis_gate_m=0.6))
    step(m, obs(0.9), t=0.0)
    c = step(m, obs(0.5), t=0.1)
    assert c.phase == "AXIS_ALIGN"


def test_timeout_aborts():
    m = MarkerApproach(MarkerDriveConfig(timeout_s=1.0))
    step(m, obs(1.5), t=0.0)
    c = step(m, obs(1.5), t=2.0)
    assert c.phase == "ABORT" and c.reason == "timeout"


# ---------------------------------------------------------------- 축 정렬

def _to_axis_align(cfg=None):
    m = MarkerApproach(cfg or MarkerDriveConfig())
    step(m, obs(0.9), t=0.0)
    step(m, obs(0.5), t=0.1)
    return m


def test_axis_align_corrects_more_when_skewed():
    straight = step(_to_axis_align(), obs(0.5, yaw=0.0, lat=0.0), t=0.2)
    skewed = step(_to_axis_align(), obs(0.5, yaw=15.0, lat=0.12), t=0.2)
    assert abs(skewed.angular) > abs(straight.angular)


def test_axis_align_moves_in_pulses():
    m = _to_axis_align(MarkerDriveConfig(move_pulse_s=0.1, move_pause_s=0.9))
    moving = step(m, obs(0.4), t=0.15)     # 펄스 진행 중(0.1 ~ 0.2)
    resting = step(m, obs(0.4), t=0.5)     # 펄스 종료 후, 다음 펄스는 1.1 부터
    assert moving.linear > 0
    assert resting.linear == 0.0


def test_reaching_stop_distance_while_visible_is_done():
    m = _to_axis_align(MarkerDriveConfig(stop_m=0.10, front_offset_m=0.0))
    c = step(m, obs(0.09), t=0.2)
    assert c.done and c.phase == "DONE" and c.reason == "reached"


def test_front_offset_shifts_stop_point():
    """정지 판정은 마커~최전방 기준이므로 오프셋만큼 더 일찍 선다."""
    m = _to_axis_align(MarkerDriveConfig(stop_m=0.10, front_offset_m=0.05))
    c = step(m, obs(0.14), t=0.2)
    assert c.done and c.reason == "reached"


def test_yaw_misalignment_blocks_blind_push():
    """lateral 이 0이어도 yaw 가 틀어져 있으면 무시각 구간에 들어가면 안 된다.

    들어가면 남은 거리를 30° 비스듬히 밀고 들어간다.
    """
    m = _to_axis_align(MarkerDriveConfig(pose_yaw_tol_deg=8.0, lost_near_m=0.25))
    c = step(m, obs(0.24, yaw=30.0, lat=0.0), t=0.2)
    assert c.phase == "AXIS_ALIGN"
    assert abs(c.angular) > 0


def test_aligned_and_near_enters_blind_push():
    m = _to_axis_align(MarkerDriveConfig(pose_yaw_tol_deg=8.0, lost_near_m=0.25))
    c = step(m, obs(0.24, yaw=2.0, lat=0.01), t=0.2)
    assert c.phase == "BLIND_PUSH"


def test_align_stall_aborts_on_lateral():
    m = _to_axis_align(MarkerDriveConfig(align_stall_s=0.5, pose_axis_tol_m=0.08))
    c = None
    for i in range(20):
        c = step(m, obs(0.18, lat=0.15), t=0.2 + 0.1 * i)
        if c.done:
            break
    assert c.phase == "ABORT" and c.reason == "align_stall"


def test_align_stall_aborts_on_yaw_only():
    """yaw 만 계속 안 맞는 경우도 잡아야 한다(lateral 은 0)."""
    m = _to_axis_align(MarkerDriveConfig(align_stall_s=0.5, pose_yaw_tol_deg=8.0))
    c = None
    for i in range(20):
        c = step(m, obs(0.18, yaw=45.0, lat=0.0), t=0.2 + 0.1 * i)
        if c.done:
            break
    assert c.phase == "ABORT" and c.reason == "align_stall"


# ---------------------------------------------------------------- 상실 분기

def test_lost_far_returns_to_search():
    m = MarkerApproach(MarkerDriveConfig(lost_grace=3, lost_near_m=0.25))
    step(m, obs(0.9), t=0.0)
    c = None
    for i in range(3):
        c = step(m, t=0.1 * (i + 1))
    assert c.phase == "SEARCH"


def test_lost_near_enters_blind_push():
    m = MarkerApproach(MarkerDriveConfig(lost_grace=3, lost_near_m=0.25))
    step(m, obs(0.5), t=0.0)
    step(m, obs(0.18), t=0.1)
    c = None
    for i in range(3):
        c = step(m, t=0.2 + 0.1 * i)
    assert c.phase == "BLIND_PUSH"


# ---------------------------------------------------------------- 무시각 전진

def test_blind_push_travels_remaining_then_done():
    m = MarkerApproach(MarkerDriveConfig(lost_grace=1, lost_near_m=0.25,
                                         stop_m=0.10, front_offset_m=0.0))
    step(m, obs(0.5), t=0.0)
    step(m, obs(0.16), t=0.1)
    c = step(m, t=0.2, fwd=0.0)
    assert c.phase == "BLIND_PUSH" and c.linear > 0
    c = step(m, t=0.3, fwd=0.03)
    assert not c.done                       # 0.06 중 0.03 만 갔다
    c = step(m, t=0.4, fwd=0.06)
    assert c.done and c.phase == "DONE"


def test_blind_push_uses_displacement_not_path_length():
    """제자리 진동으로 누적거리가 늘어도 완료로 읽히면 안 된다."""
    m = MarkerApproach(MarkerDriveConfig(lost_grace=1, lost_near_m=0.25,
                                         stop_m=0.10, front_offset_m=0.0,
                                         no_progress_s=100.0))
    step(m, obs(0.5), t=0.0)
    step(m, obs(0.16), t=0.1)
    step(m, t=0.2, fwd=0.0)
    c = None
    for i in range(10):                     # 앞뒤로 0.02m 왕복 = 누적 0.4m, 순전진 0
        c = step(m, t=0.3 + 0.1 * i, fwd=0.02 if i % 2 == 0 else 0.0)
    assert not c.done
    assert c.phase == "BLIND_PUSH"


def test_blind_push_ignores_sideways_motion():
    """옆으로 밀린 거리는 전진이 아니다 — 벽까지 남은 거리가 줄지 않는다.

    시작점 대비 직선거리로 재면 옆으로 6cm 밀린 것이 '도달'로 읽힌다.
    """
    m = MarkerApproach(MarkerDriveConfig(lost_grace=1, lost_near_m=0.25,
                                         stop_m=0.10, front_offset_m=0.0,
                                         no_progress_s=100.0))
    step(m, obs(0.5), t=0.0)
    step(m, obs(0.16), t=0.1)               # 남은 목표 0.06m
    step(m, t=0.2, fwd=0.0)
    c = step(m, t=0.3, fwd=0.0)             # 옆으로만 밀림 → 전진 성분 0
    assert not c.done and c.phase == "BLIND_PUSH"


def test_blind_push_subtracts_coasting_during_lost_grace():
    """상실 유예 동안 관성으로 더 간 거리는 목표에서 빠져야 한다.

    진입 시점을 기준 삼으면 그만큼 더 밀어 목표를 지나친다.
    """
    m = MarkerApproach(MarkerDriveConfig(lost_grace=3, lost_near_m=0.25,
                                         stop_m=0.10, front_offset_m=0.0))
    step(m, obs(0.5), t=0.0)
    step(m, obs(0.16), t=0.1, fwd=0.00)     # 마지막 관측: 남은 목표 0.06m
    step(m, t=0.2, fwd=0.02)                # 유예 중 관성 2cm
    step(m, t=0.3, fwd=0.04)                # 유예 중 관성 총 4cm
    c = step(m, t=0.4, fwd=0.06)            # 관측 시점 대비 6cm → 도달
    assert c.done and c.phase == "DONE" and c.reason == "reached"


def test_blind_push_blocked_when_no_progress():
    m = MarkerApproach(MarkerDriveConfig(lost_grace=1, lost_near_m=0.25, no_progress_s=0.5))
    step(m, obs(0.5), t=0.0)
    step(m, obs(0.18), t=0.1)
    c = None
    for i in range(10):
        c = step(m, t=0.2 + 0.1 * i, fwd=0.0)
        if c.done:
            break
    assert c.phase == "ABORT" and c.reason == "blocked"


# ---------------------------------------------------------------- 안전

def test_scan_guard_stops_immediately():
    c = step(_to_axis_align(), obs(0.3), front=0.03, t=0.5)
    assert c.done and c.reason == "scan_guard"
    assert c.linear == 0.0 and c.angular == 0.0


def test_nan_scan_does_not_bypass_guard_silently():
    """NaN 은 모든 비교가 False 라 '안전'처럼 통과한다. '값 없음'으로 취급해야 한다."""
    m = _to_axis_align()
    c = step(m, obs(0.3), front=float("nan"), t=0.5)
    assert c.phase == "AXIS_ALIGN"          # 계속 진행하되 근접 판정에는 안 쓴다
    assert not c.done


def test_nan_observation_is_treated_as_no_detection():
    m = MarkerApproach(MarkerDriveConfig())
    c = step(m, obs(float("nan")))
    assert c.phase == "SEARCH"


def test_bad_odom_aborts():
    m = MarkerApproach(MarkerDriveConfig())
    c = step(m, obs(1.0), yaw=float("nan"))
    assert c.phase == "ABORT" and c.reason == "bad_odom"


def test_finished_machine_keeps_returning_zero():
    m = _to_axis_align()
    step(m, obs(0.05), t=0.2)               # DONE
    c = step(m, obs(0.5), t=0.3)
    assert c.done and c.linear == 0.0 and c.angular == 0.0


# ------------------------------------------------- codex 적대적 리뷰 회귀 테스트

def test_loss_while_misaligned_does_not_enter_blind_push():
    """보이는 동안엔 yaw 를 확인하는데 상실 경로가 그 검사를 우회하면 안 된다.

    45° 틀어진 채로 마커를 잃고 무시각 직진에 들어가면 비뚤게 박는다.
    """
    m = _to_axis_align(MarkerDriveConfig(lost_grace=1, lost_near_m=0.25,
                                         align_stall_s=100.0))
    step(m, obs(0.18, yaw=45.0, lat=0.15), t=0.2)
    c = step(m, t=0.3)
    assert c.phase != "BLIND_PUSH"
    assert c.phase == "SEARCH"


def test_stop_distance_while_misaligned_rotates_instead_of_finishing():
    """거리가 맞아도 비뚤면 완료가 아니다 — 전진을 멈추고 제자리에서 맞춘다."""
    m = _to_axis_align(MarkerDriveConfig(stop_m=0.10, front_offset_m=0.0,
                                         pose_yaw_tol_deg=8.0, align_stall_s=100.0))
    c = step(m, obs(0.09, yaw=45.0, lat=0.20), t=0.2)
    assert not c.done
    assert c.phase == "AXIS_ALIGN" and c.reason == "final_align"
    assert c.linear == 0.0 and abs(c.angular) > 0


def test_align_stall_budget_resets_after_an_aligned_frame():
    """한 번 맞았다가 다시 틀어지면 정렬 예산을 새로 받아야 한다."""
    # z 는 lost_near_m(0.25) 보다 멀게 둔다 — 가까우면 정렬된 순간 정상적으로
    # BLIND_PUSH 로 넘어가 버려서 이 테스트가 검증하려는 지점을 못 본다.
    m = _to_axis_align(MarkerDriveConfig(align_stall_s=0.5, pose_yaw_tol_deg=8.0))
    step(m, obs(0.40, yaw=45.0), t=0.5)            # 미정렬
    step(m, obs(0.40, yaw=1.0, lat=0.0), t=0.55)   # 정렬됨 → 예산 갱신
    c = step(m, obs(0.40, yaw=45.0), t=0.70)       # 다시 미정렬(0.15s 경과)
    assert c.phase == "AXIS_ALIGN"


def test_align_stall_timer_does_not_leak_across_search():
    """상실→탐색→재획득한 정렬 시도가 즉시 중단되면 안 된다."""
    m = MarkerApproach(MarkerDriveConfig(lost_grace=1, align_stall_s=0.5,
                                         pose_axis_tol_m=0.08))
    step(m, obs(0.50, lat=0.10), t=0.0)      # AXIS_ALIGN 진입(미정렬)
    step(m, t=0.1)                            # 상실 → SEARCH
    c = step(m, obs(0.50, lat=0.10), t=1.0)  # 재획득
    assert c.phase == "AXIS_ALIGN" and not c.done


def test_steering_state_is_reset_after_reacquisition():
    """상실 구간을 넘어 LPF 가 살아남으면 재획득 첫 틱이 옛 방향으로 나간다."""
    m = MarkerApproach(MarkerDriveConfig(lost_grace=1, axis_gate_m=0.6))
    right = step(m, obs(1.5, ex=+0.5), t=0.0)
    step(m, t=0.1)                            # 상실 → SEARCH (조향 상태 초기화)
    left = step(m, obs(1.5, ex=-0.5), t=1.0)
    assert right.angular * left.angular < 0   # 즉시 반대로 꺾여야 한다


def test_dt_uses_real_elapsed_time():
    """적분은 실제 경과 시간을 받아야 한다. 1/loop_hz 고정이면 시간을 속인다."""
    cfg = MarkerDriveConfig(steer_ki=0.4, steer_kd=0.0, loop_hz=12.0,
                            steer_ang_max=0.3, steer_ang_min=0.0)
    fast = MarkerApproach(cfg)
    slow = MarkerApproach(cfg)
    for i in range(3):
        f = step(fast, obs(1.5, ex=0.3), t=0.08 * i)
        s = step(slow, obs(1.5, ex=0.3), t=1.00 * i)
    assert abs(s.angular) > abs(f.angular)    # 느린 루프가 적분을 더 쌓는다
