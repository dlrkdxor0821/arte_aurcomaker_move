"""CLI 기본값이 설정 기본값에서 갈라지지 않는지 지킨다.

argparse 에 숫자를 복사해 두면, 나중에 MarkerDriveConfig 를 고쳐도 CLI 는 옛 값으로
돈다. 설정 테스트는 여전히 통과하므로 아무도 못 알아챈다 — 그래서 이 파일이 있다.

drive 모듈은 rclpy 를 임포트하므로, ROS 없이도 돌도록 최소 스텁을 끼운다.
"""
import sys
import types

import pytest

from marker.config import MarkerDriveConfig

_ROS_STUBS = ("rclpy", "rclpy.qos", "geometry_msgs", "geometry_msgs.msg",
              "nav_msgs", "nav_msgs.msg", "sensor_msgs", "sensor_msgs.msg")


@pytest.fixture
def drive_module(monkeypatch):
    for name in _ROS_STUBS:
        monkeypatch.setitem(sys.modules, name, types.ModuleType(name))
    sys.modules["geometry_msgs.msg"].Twist = object
    sys.modules["nav_msgs.msg"].Odometry = object
    sys.modules["sensor_msgs.msg"].LaserScan = object
    sys.modules["rclpy.qos"].qos_profile_sensor_data = None
    monkeypatch.delitem(sys.modules, "marker.drive", raising=False)
    monkeypatch.delitem(sys.modules, "marker.odom", raising=False)
    monkeypatch.delitem(sys.modules, "marker.scan", raising=False)
    import marker.drive as drive
    return drive


FIELDS = ("marker_id", "marker_len_m", "dict_name", "stop_m", "front_offset_m",
          "steer_sign", "axis_gate_m", "yaw_offset_deg", "pose_yaw_tol_deg",
          "scan_guard_m", "lin_homing", "lin_pulse", "ang_search", "steer_ang_max",
          "pose_kp_lat", "sensor_timeout_s", "search_step_deg", "search_span_deg",
          "timeout_s", "loop_hz")


def test_cli_defaults_match_config_defaults(drive_module):
    from_cli = drive_module._config(drive_module._parse(["drive"]))
    from_config = MarkerDriveConfig().clamped()
    mismatched = [f for f in FIELDS
                  if getattr(from_cli, f) != getattr(from_config, f)]
    assert not mismatched, f"CLI 가 설정 기본값에서 갈라졌다: {mismatched}"


def test_explicit_flags_override(drive_module):
    cfg = drive_module._config(drive_module._parse(
        ["drive", "--steer-sign", "-1", "--axis-gate", "0.45", "--stop-m", "0.12",
         "--steer-ang-max", "0.2", "--sensor-timeout", "0.8"]))
    assert cfg.steer_sign == -1.0
    assert cfg.axis_gate_m == 0.45
    assert cfg.stop_m == 0.12
    assert cfg.steer_ang_max == 0.2
    assert cfg.sensor_timeout_s == 0.8


def test_modes_and_scan_dicts_parse(drive_module):
    assert drive_module._parse(["detect", "--scan-dicts"]).scan_dicts is True
    assert drive_module._parse(["stop"]).mode == "stop"
    assert drive_module._parse(["drive", "--source", "/tmp/x.avi"]).source == "/tmp/x.avi"
