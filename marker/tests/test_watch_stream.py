"""MJPEG 스트림 — 헤드리스 로봇에서 '안 잡힌다'를 눈으로 보는 유일한 통로다.

경계 문자열이나 헤더가 틀리면 브라우저는 에러 없이 **빈 화면**만 보여 준다.
카메라가 죽은 건지 스트림이 깨진 건지 구분이 안 되므로 여기서 형식을 잡아 둔다.
"""
import urllib.request

import numpy as np

from marker.detect import detect_all, make_marker_image
from marker.watch import _latest, _serve_mjpeg


def test_stream_sends_jpeg_frames_with_boundary():
    srv = _serve_mjpeg(0)                       # 0 = 빈 포트를 OS 가 고른다
    try:
        _latest["jpg"] = b"\xff\xd8\xff\xe0fake-jpeg"
        port = srv.server_address[1]
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/", timeout=5) as r:
            assert "multipart/x-mixed-replace" in r.headers["Content-Type"]
            chunk = r.read(64)
    finally:
        srv.shutdown()
        srv.server_close()
        _latest["jpg"] = None
    assert b"--frame" in chunk
    assert b"\xff\xd8" in chunk, "JPEG 매직이 없다 — 브라우저가 빈 화면만 본다"


def test_detect_all_reports_every_id_not_just_the_target():
    """'아무것도 안 보인다'와 '보이는데 ID 가 다르다'를 가르는 정보원이다."""
    frame = np.full((240, 320), 255, dtype=np.uint8)
    frame[40:140, 40:140] = make_marker_image("DICT_5X5_100", 7, 100)
    found = detect_all(frame, "DICT_5X5_100")
    assert [mid for mid, _ in found] == [7]
    assert found[0][1].shape == (4, 2)
    assert not detect_all(frame, "DICT_4X4_50"), "사전이 다르면 안 잡혀야 한다"
