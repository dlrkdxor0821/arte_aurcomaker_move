#!/usr/bin/env python3
"""마커 검출 관찰 — 모터를 건드리지 않는다.

용도 둘:
  1. 마커가 몇 m 부터 잡히는지, yaw 가 어느 거리부터 안정되는지 → --axis-gate 값
  2. **안 잡힐 때 왜 안 잡히는지** — 헤드리스 로봇에서는 --http 로 본다

    python3 -m marker.watch --source 1 --slot back --rotate 0 --http 8088
    →  노트북 브라우저에서 http://<로봇IP>:8088

화면이 필요한 이유: '검출 0개'는 원인이 다섯 가지다(사전 틀림·ID 다름·너무 작음·
너무 어두움·아예 화면 밖). 숫자만 보면 다섯이 전부 똑같은 한 줄로 보인다.
그래서 이 도구는 잡힌 마커를 **전부** 그리고(대상은 초록, 나머지는 주황),
아무것도 못 잡으면 1초에 한 번 모든 사전을 훑어 화면에 적는다.
"""
import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

from .calib import load_calib
from .camera import open_camera
from .config import MarkerDriveConfig
from .detect import detect_all, detect_marker, scan_dicts

_D = MarkerDriveConfig()      # 기본값의 유일한 출처 (drive.py 와 같은 이유)

_latest = {"jpg": None}       # 최신 프레임 한 장. 스트림 스레드가 여기서만 읽는다.


def _serve_mjpeg(port: int) -> ThreadingHTTPServer:
    """최신 프레임을 붙는 클라이언트에게 흘린다. 서버 객체를 돌려준다.

    HTTP 쪽에서 카메라를 당기지 않는 이유: 브라우저를 닫는 순간 검출 루프까지
    멈춰서, 정작 보려던 로그가 같이 끊긴다. 캡처는 본 루프가 계속 돌고 여기는
    마지막 장만 본다.
    """
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    jpg = _latest["jpg"]
                    if jpg is not None:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                         b"Content-Length: %d\r\n\r\n" % len(jpg))
                        self.wfile.write(jpg + b"\r\n")
                    time.sleep(0.08)     # ponytail: 고정 12fps. 프레임 이벤트 대기는 과하다
            except (BrokenPipeError, ConnectionResetError):
                pass                     # 브라우저 탭을 닫은 것뿐이다

        def log_message(self, *a):       # 매 프레임 로그를 찍으면 검출 출력이 묻힌다
            pass

    srv = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


def _annotate(frame, hits, text: str) -> None:
    """잡힌 마커를 전부 그리고 상태 한 줄을 얹는다(프레임을 제자리에서 고친다)."""
    for mid, pts, is_target in hits:
        color = (0, 255, 0) if is_target else (0, 165, 255)
        cv2.polylines(frame, [pts.astype(int)], True, color, 2)
        cv2.putText(frame, f"id={mid}", tuple(pts[0].astype(int)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    cv2.putText(frame, text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (0, 255, 0) if any(t for _, _, t in hits) else (0, 0, 255), 1)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="마커 검출 관찰(모터 무접촉)")
    ap.add_argument("--source", default="0", help="'csi' 또는 USB 인덱스")
    ap.add_argument("--slot", default="back", choices=["front", "front0", "back"])
    ap.add_argument("--rotate", type=int, default=0, choices=[0, 180],
                    help="90/270 은 캘리브레이션이 같이 안 돌아 지원하지 않는다")
    ap.add_argument("--marker-id", type=int, default=_D.marker_id)
    ap.add_argument("--marker-m", type=float, default=_D.marker_len_m)
    ap.add_argument("--dict", dest="dict_name", default=_D.dict_name)
    ap.add_argument("--stop-m", type=float, default=_D.stop_m)
    ap.add_argument("--scan-dicts", dest="scan_dicts", action="store_true")
    ap.add_argument("--http", type=int, default=0, metavar="PORT",
                    help="MJPEG 스트림 포트. 헤드리스 로봇은 이걸로 본다")
    ap.add_argument("--no-window", action="store_true", help="창 없이 텍스트만")
    a = ap.parse_args(argv)
    window = not a.no_window and not a.http     # 스트림을 켰다면 화면은 없는 환경이다

    K, dist, (w, h) = load_calib(a.slot, a.rotate)
    cam = open_camera(a.source, width=w, height=h, rotate=a.rotate)
    if a.http:
        _serve_mjpeg(a.http)
        print(f"[ok] MJPEG http://<로봇IP>:{a.http}  (브라우저로 열어라)")
    print(f"[ok] source={a.source} slot={a.slot} {w}x{h} id={a.marker_id} dict={a.dict_name}")
    next_scan = 0.0
    dict_hint = ""
    try:
        while True:
            frame = cam.get_frame()
            if frame is None:
                print("[error] 프레임 없음")
                return 2
            if a.scan_dicts:
                hits = []
                text = "사전: " + (", ".join(f"{n}{ids}" for n, ids in scan_dicts(frame)) or "없음")
            else:
                # 관찰 도구라 검출을 두 번 돌린다(자세 1회 + 그리기 1회). 320x240 에서
                # 30ms 남짓이고, 제어 루프가 아니라 사람이 보는 화면이라 문제되지 않는다.
                found = detect_all(frame, a.dict_name)
                hits = [(mid, pts, mid == a.marker_id) for mid, pts in found]
                obs = detect_marker(frame, K, dist, marker_len_m=a.marker_m,
                                    target_id=a.marker_id, dict_name=a.dict_name)
                bright = int(frame.mean())
                if obs is not None:
                    text = obs.describe(a.stop_m)
                elif found:
                    # 사전은 맞다. 대상 ID 가 없거나, 코너/재투영 게이트에서 걸렸다.
                    text = f"id {[m for m, _, _ in hits]} 는 보이는데 {a.marker_id} 는 아니다"
                else:
                    # 아무것도 없다 = 사전이 틀렸거나 화면 밖이거나 너무 어둡다.
                    # 사전 훑기는 7개를 다 도니 1초에 한 번만.
                    if time.monotonic() >= next_scan:
                        found_d = scan_dicts(frame)
                        dict_hint = (", ".join(f"{n}{i}" for n, i in found_d)
                                     if found_d else "어느 사전에도 없음")
                        next_scan = time.monotonic() + 1.0
                    text = f"검출 0 | 사전훑기: {dict_hint}"
                text += f" | 밝기 {bright}"
            print(text, flush=True)
            _annotate(frame, hits, text)
            if a.http:
                ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    _latest["jpg"] = buf.tobytes()
            if window:
                cv2.imshow("marker-watch", frame)
                if cv2.waitKey(30) & 0xFF == ord("q"):
                    break
            else:
                time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        cam.close()
        if window:
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
