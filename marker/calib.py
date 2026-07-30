"""슬롯 이름이 곧 캘리브레이션 선택 키다.

주점(cx)이 회전 여부에 따라 80px 넘게 다르다(rot180: 278.2 vs 비회전: 360.8).
잘못 물리면 거리는 그럴듯한데 정렬만 한쪽으로 흐른다 — 가장 찾기 어려운 버그다.
"""
import pathlib

import numpy as np

CALIB_DIR = pathlib.Path(__file__).resolve().parents[1] / "config" / "camera"
CALIB_FILES = {
    "front": "picam_640x480_rot180.npz",   # picam CSI, 거꾸로 장착 → --rotate 180
    "front0": "picam_640x480.npz",         # 같은 CSI 카메라를 바로 세워 달았을 때 → --rotate 0
    "back": "usb_320x240.npz",             # 뒷캠 USB 웹캠 (/dev/video1, 320x240) → --rotate 0
}


def load_calib(slot: str, rotate: int | None = None):
    """(K, dist, (w, h)) 를 돌려준다. rotate 를 주면 짝이 맞는지 확인하고 아니면 막는다.

    해상도를 같이 주는 이유: K 는 캘리브한 해상도에서만 맞는다. 320x240 으로 잰
    카메라를 640x480 으로 열면 fx·cx 가 그대로 두 배 틀려 거리가 절반으로 읽힌다 —
    죽지 않고 그럴듯한 숫자가 나오는 종류의 오류다. 그래서 호출자가 해상도를
    고르게 두지 않고 캘리브 파일이 정한다.

    회전본은 npz 에 rotation_deg 를 갖고 있다. 없으면 원본(0도)이다.
    짝이 틀리면 거리는 그럴듯한데 좌우만 흐르는, 가장 찾기 어려운 증상이 된다.
    """
    if slot not in CALIB_FILES:
        raise ValueError(f"슬롯은 {'/'.join(CALIB_FILES)} 중 하나여야 한다: {slot}")
    path = CALIB_DIR / CALIB_FILES[slot]
    if not path.exists():
        raise FileNotFoundError(f"캘리브레이션이 없다: {path}")
    data = np.load(path)
    made_for = int(data["rotation_deg"]) if "rotation_deg" in data else 0
    if rotate is not None and made_for != rotate:
        raise SystemExit(
            f"슬롯 '{slot}'({path.name})은 --rotate {made_for} 로 캘리브된 값인데 "
            f"--rotate {rotate} 로 돌리려 한다.\n"
            f"cx 가 80px 넘게 달라 좌우 오차가 통째로 틀어진다. "
            f"짝: front=180, front0=0, back=0")
    w, h = (int(v) for v in data["image_size"])
    return data["camera_matrix"], data["dist_coeffs"], (w, h)
