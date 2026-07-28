#!/usr/bin/env python3
"""노트북에서 카메라 영상에 마커 검출 오버레이를 씌워 본다. 모터를 건드리지 않는다.

용도: 마커가 몇 m 부터 잡히는지, yaw 가 어느 거리부터 안정되는지 보고
      로봇 쪽 --axis-gate 값을 정하는 것.

노트북에 붙은 USB 캠으로 직접 볼 수도 있고(--source 0),
로봇에서 같은 마커를 같은 조건으로 확인할 때는 로봇에서 --source csi 로 돌린다.
"""
import argparse

import cv2

from .calib import load_calib
from .camera import open_camera
from .config import MarkerDriveConfig
from .detect import detect_marker, scan_dicts

_D = MarkerDriveConfig()      # 기본값의 유일한 출처 (drive.py 와 같은 이유)


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
    ap.add_argument("--no-window", action="store_true", help="창 없이 텍스트만")
    a = ap.parse_args(argv)

    K, dist = load_calib(a.slot, a.rotate)
    cam = open_camera(a.source, rotate=a.rotate)
    print(f"[ok] source={a.source} slot={a.slot} id={a.marker_id} dict={a.dict_name}")
    try:
        while True:
            frame = cam.get_frame()
            if frame is None:
                print("[error] 프레임 없음")
                return 2
            if a.scan_dicts:
                hits = scan_dicts(frame)
                text = "사전: " + (", ".join(f"{n}{ids}" for n, ids in hits) if hits else "없음")
                obs = None
            else:
                obs = detect_marker(frame, K, dist, marker_len_m=a.marker_m,
                                    target_id=a.marker_id, dict_name=a.dict_name)
                text = "marker: --" if obs is None else obs.describe(a.stop_m)
            print(text, flush=True)
            if not a.no_window:
                cv2.putText(frame, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 255, 0) if obs else (0, 0, 255), 2)
                cv2.imshow("marker-watch", frame)
                if cv2.waitKey(30) & 0xFF == ord("q"):
                    break
            else:
                cv2.waitKey(1)
    except KeyboardInterrupt:
        pass
    finally:
        cam.close()
        if not a.no_window:
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
