"""카메라 어댑터 — 장치 없이 확인 가능한 계약만 본다."""
import pytest

from marker.camera import open_camera


@pytest.mark.parametrize("rotate", [90, 270])
def test_rotations_without_matching_calibration_are_rejected(rotate):
    """90/270 은 픽셀만 돌고 K 는 안 돈다.

    실측: 오른쪽 100px 치우친 마커가 회전 후 ex≈0(정중앙)으로 읽힌다 — 조향이
    이탈을 못 보고 직진한다. 세로 장착을 쓰려면 그 자세로 새로 캘리브해야 한다.
    """
    with pytest.raises(SystemExit) as exc:
        open_camera("0", rotate=rotate)
    assert "rotate" in str(exc.value)


def test_missing_video_file_fails_loudly():
    with pytest.raises(SystemExit) as exc:
        open_camera("/nonexistent/does-not-exist.avi", rotate=0)
    assert "영상 소스를 열 수 없다" in str(exc.value)
