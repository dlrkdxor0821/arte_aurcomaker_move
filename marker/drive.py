#!/usr/bin/env python3
"""마커 주행 CLI — 로봇에서 실행한다.

    python3 -m marker.drive drive              마커 찾아 앞 10cm 까지
    python3 -m marker.drive detect             모터 정지, 검출값만 출력
    python3 -m marker.drive detect --scan-dicts   어떤 사전의 마커인지 훑어본다
    python3 -m marker.drive stop               /cmd_vel 0 발행 후 종료

전제: 로봇 구동 노드(/cmd_vel 구독, /odom·/scan 발행)가 떠 있어야 한다.
      다른 스택(추종·영상송출·nav2)은 떠 있으면 안 된다 — 카메라와 /cmd_vel 을 다툰다.

`drive` 모드는 제어 루프에 들어가기 전에 /odom·/scan 이 최소 한 번 들어올 때까지
최대 --sensor-wait 초 기다린다. front_m=None 을 '근접 위험 없음'으로 읽는 상태기계
특성상, /scan 이 아직 안 들어온 채로 움직이기 시작하면 근접 안전장치가 꺼진 첫 몇
사이클을 그냥 지나치게 된다. `detect` 모드는 모터를 건드리지 않으므로 기다리지
않고 센서 상태만 출력한다.
"""
import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import Twist

from .approach import MarkerApproach
from .calib import load_calib
from .camera import open_camera
from .config import MarkerDriveConfig
from .detect import detect_marker, scan_dicts
from .odom import OdomTracker
from .scan import ScanWatch


def _parse(argv):
    ap = argparse.ArgumentParser(description="ArUco 마커 주행 (nav2 미사용)")
    ap.add_argument("mode", choices=["drive", "detect", "stop"])
    ap.add_argument("--source", default="csi", help="'csi' 또는 USB 인덱스('0')")
    ap.add_argument("--slot", default="front", choices=["front", "back"],
                    help="캘리브 선택 키")
    ap.add_argument("--rotate", type=int, default=180, choices=[0, 90, 180, 270])
    ap.add_argument("--scan-dicts", dest="scan_dicts", action="store_true",
                    help="detect 모드에서 어떤 사전의 마커인지 훑어본다")
    ap.add_argument("--marker-id", type=int, default=1)
    ap.add_argument("--marker-m", type=float, default=0.07)
    ap.add_argument("--dict", dest="dict_name", default="DICT_5X5_100")
    ap.add_argument("--stop-m", type=float, default=0.10, help="마커~로봇 최전방 목표")
    ap.add_argument("--front-offset", type=float, default=0.0, help="카메라~로봇 최전방")
    ap.add_argument("--steer-sign", type=float, default=1.0, help="반대로 돌면 -1")
    ap.add_argument("--axis-gate", type=float, default=0.6, help="yaw 신뢰 시작 거리")
    ap.add_argument("--lin-homing", type=float, default=0.12)
    ap.add_argument("--lin-pulse", type=float, default=0.08)
    ap.add_argument("--ang-search", type=float, default=0.35)
    ap.add_argument("--search-step-deg", type=float, default=20.0)
    ap.add_argument("--search-span-deg", type=float, default=60.0)
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--loop-hz", type=float, default=12.0)
    ap.add_argument("--cmd-topic", default="/cmd_vel")
    ap.add_argument("--sensor-wait", type=float, default=5.0,
                    help="drive 모드에서 /odom·/scan 이 들어올 때까지 기다리는 최대 초")
    return ap.parse_args(argv)


def _config(a) -> MarkerDriveConfig:
    return MarkerDriveConfig(
        marker_id=a.marker_id, marker_len_m=a.marker_m, dict_name=a.dict_name,
        stop_m=a.stop_m, front_offset_m=a.front_offset, steer_sign=a.steer_sign,
        axis_gate_m=a.axis_gate, lin_homing=a.lin_homing, lin_pulse=a.lin_pulse,
        ang_search=a.ang_search, search_step_deg=a.search_step_deg,
        search_span_deg=a.search_span_deg, timeout_s=a.timeout, loop_hz=a.loop_hz,
    ).clamped()


def _publish(pub, lin: float, ang: float) -> None:
    t = Twist()
    t.linear.x = float(lin)
    t.angular.z = float(ang)
    pub.publish(t)


def _wait_sensors(node, odom: OdomTracker, watch: ScanWatch, timeout_s: float) -> bool:
    """odom.ready 와 watch.ready 가 둘 다 True 가 될 때까지 최대 timeout_s 초 spin 한다."""
    deadline = time.monotonic() + timeout_s
    while not (odom.ready and watch.ready) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    return odom.ready and watch.ready


def main(argv=None) -> int:
    a = _parse(argv if argv is not None else sys.argv[1:])
    cfg = _config(a)
    rclpy.init()
    node = rclpy.create_node("marker_drive")
    pub = node.create_publisher(Twist, a.cmd_topic, 10)

    if a.mode == "stop":
        for _ in range(5):
            _publish(pub, 0.0, 0.0)
            time.sleep(0.05)
        node.destroy_node()
        rclpy.shutdown()
        print("[ok] 정지 명령 발행 완료")
        return 0

    K, dist = load_calib(a.slot)
    cam = open_camera(a.source, rotate=a.rotate)
    odom = OdomTracker(node)
    watch = ScanWatch(node)
    machine = MarkerApproach(cfg)
    period = 1.0 / cfg.loop_hz
    print(f"[ok] mode={a.mode} source={a.source} slot={a.slot} id={cfg.marker_id} "
          f"marker={cfg.marker_len_m}m stop={cfg.stop_m}m(+{cfg.front_offset_m}) "
          f"gate={cfg.axis_gate_m}m sign={cfg.steer_sign:+.0f} dict={cfg.dict_name}")
    try:
        if a.mode == "drive":
            if not _wait_sensors(node, odom, watch, a.sensor_wait):
                missing = [name for name, ok in (("/odom", odom.ready), ("/scan", watch.ready))
                          if not ok]
                _publish(pub, 0.0, 0.0)
                print(f"[error] 센서 준비 안 됨({a.sensor_wait:.1f}초 대기) — "
                      f"다음 토픽이 안 들어온다: {', '.join(missing)}")
                return 3
            print("[ok] 센서 준비 완료 (odom, scan)")
        else:
            rclpy.spin_once(node, timeout_sec=0.1)
            print(f"[센서] odom_ready={odom.ready} scan_ready={watch.ready}")

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            frame = cam.get_frame()
            if frame is None:
                _publish(pub, 0.0, 0.0)
                print("[error] 카메라 프레임이 없다 — 장치를 확인해라")
                return 2

            if a.mode == "detect":
                if a.scan_dicts:
                    hits = scan_dicts(frame)
                    print("검출된 사전: " + (", ".join(f"{n}{ids}" for n, ids in hits)
                                             if hits else "없음"))
                else:
                    o = detect_marker(frame, K, dist, marker_len_m=cfg.marker_len_m,
                                      target_id=cfg.marker_id, dict_name=cfg.dict_name)
                    front = f"{watch.front_m:.3f}m" if watch.front_m is not None else "--"
                    status = f"odom_ready={odom.ready} scan_ready={watch.ready}"
                    if o is None:
                        print(f"marker: --   scan_front={front}  {status}")
                    else:
                        remain = o.z_m - (cfg.stop_m + cfg.front_offset_m)
                        print(f"marker: z={o.z_m:.3f}m ex={o.ex:+.3f} yaw={o.yaw_deg:+.1f} "
                              f"lat={o.lateral_m:+.3f} size={o.size_frac:.2f} | "
                              f"앞면까지 남음 {remain:+.3f}m  scan_front={front}  {status}")
                time.sleep(max(period, 0.5))
                continue

            o = detect_marker(frame, K, dist, marker_len_m=cfg.marker_len_m,
                              target_id=cfg.marker_id, dict_name=cfg.dict_name)
            cmd = machine.step(o, yaw_deg=odom.yaw_deg, travel_m=odom.travel_m,
                               pos_xy=odom.pos_xy, front_m=watch.front_m,
                               now_s=time.monotonic())
            _publish(pub, cmd.linear, cmd.angular)
            print(f"[{cmd.phase:<10}] lin={cmd.linear:+.3f} ang={cmd.angular:+.3f} "
                  f"reason={cmd.reason}")
            if cmd.done:
                _publish(pub, 0.0, 0.0)
                ok = cmd.phase == "DONE"
                print("[ok] 도착" if ok else f"[fail] 중단: {cmd.reason}")
                return 0 if ok else 1
            time.sleep(period)
        return 0
    except KeyboardInterrupt:
        print("\n[stop] 사용자 중단 — 모터 정지")
        return 130
    finally:
        for _ in range(3):
            _publish(pub, 0.0, 0.0)
        cam.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
