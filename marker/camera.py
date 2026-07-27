"""프레임 획득 — 이 파일이 이음매다.

1단계(이 레포, 단독 실행)는 장치를 직접 연다. 다른 스택이 안 떠 있으므로 경합이 없다.

3단계에서 aba_project 미션 BT 로 승격하면 그때는 영상 송출 프로세스가 카메라를 잡고
있다. 장치를 직접 열면 앞캠이 'Device or resource busy' 로 죽는다. 그때는 아래
get_frame() 한 곳만 공유메모리 탭 읽기로 바꾸면 되고, 상태기계는 손대지 않는다.

CSI(picam)는 일반 VideoCapture 로 검은 화면만 나오므로 picamera2 를 거친다.
"""
import cv2

# 90/270 은 넣지 않는다. 픽셀만 돌리고 K(fx, fy, cx, cy)는 그대로 쓰게 되는데,
# 그러면 가로·세로가 뒤바뀐 영상에 640x480 캘리브를 물려 좌우 오차가 통째로 틀어진다
# (실측: 오른쪽 100px 치우친 마커가 회전 후 ex≈0, 즉 '정중앙'으로 읽힌다).
# 세로 장착 카메라를 쓰려면 그 자세로 새로 캘리브해서 slot 을 추가해야 한다.
_ROTATE = {180: cv2.ROTATE_180}


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


def _open_capture(target, width: int, height: int, label: str) -> Camera:
    """VideoCapture 로 여는 모든 소스(USB 인덱스 / 영상 파일) 공통 경로."""
    cap = cv2.VideoCapture(target)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not cap.isOpened():
        raise SystemExit(f"영상 소스를 열 수 없다: {label}")

    def read():
        ok, frame = cap.read()
        return frame if ok else None

    return Camera(read, cap.release)


def open_camera(source: str = "csi", *, width: int = 640, height: int = 480,
                rotate: int = 180) -> Camera:
    """source: 'csi' | USB 장치 인덱스('0', '1', ...) | 영상 파일 경로.

    영상 파일도 받는 이유: 실기 없이 같은 제어 루프를 그대로 돌려볼 수 있고,
    현장에서 찍어 온 영상으로 게이트·극성을 다시 맞출 수 있다. 프레임이 끝나면
    get_frame() 이 None 을 돌려주므로 호출부가 정상 종료한다.

    rotate 기본 180 은 이 Pi 의 CSI 카메라가 거꾸로 장착돼 있기 때문이며,
    config/camera/picam_640x480_rot180.npz 가 그 상태로 캘리브된 것이다.
    """
    if rotate not in (0, 180):
        raise SystemExit(
            f"--rotate {rotate} 는 지원하지 않는다(0 또는 180만).\n"
            "90/270 은 영상만 돌고 캘리브레이션은 안 돌아서 좌우 오차가 틀어진다.")
    if source == "csi":
        cam = _open_csi(width, height)
    elif source.isdigit():
        cam = _open_capture(int(source), width, height, f"USB index={source}")
    else:
        cam = _open_capture(source, width, height, f"file={source}")
    cam._rotate = rotate
    return cam
