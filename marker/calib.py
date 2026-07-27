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
