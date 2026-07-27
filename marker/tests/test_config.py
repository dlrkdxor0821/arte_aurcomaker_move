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


def test_steer_sign_is_normalized_to_plus_or_minus_one():
    assert MarkerDriveConfig(steer_sign=-0.3).clamped().steer_sign == -1.0
    assert MarkerDriveConfig(steer_sign=4.0).clamped().steer_sign == 1.0


def test_value_types_are_frozen():
    obs = MarkerObs(marker_id=1, ex=0.0, z_m=1.0,
                    yaw_deg=0.0, lateral_m=0.0, size_frac=0.1)
    cmd = Cmd(linear=0.0, angular=0.0, phase="SEARCH", done=False, reason="")
    for frozen, field in ((obs, "z_m"), (cmd, "linear")):
        try:
            setattr(frozen, field, 9.9)
        except Exception as exc:
            assert "frozen" in type(exc).__name__.lower()
        else:
            raise AssertionError(f"{type(frozen).__name__} 이 frozen 이 아니다")
