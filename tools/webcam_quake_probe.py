#!/usr/bin/env python3
"""웹캠으로 지진 판정(흔들림 횟수)을 실시간 확인하고 Q-11 값을 재는 도구 (E3 예비 측정).

판정은 rescue_perception.disaster_sign 을 그대로 쓰고, 이 도구는 OpenCV 로 프레임을
읽어 넣어 주기만 한다. 로봇·ROS 를 쓰지 않는다.

사용:
    python3 tools/webcam_quake_probe.py --csv quake_probe.csv

창에서 숫자 키로 지금 하는 상황을 표시하면 CSV 의 label 칸에 남는다.
    1 still(가만히)  2 shake(흔들기)  3 light(조명 켜기·끄기)  4 pass(앞 지나가기)
    5 cover(렌즈 가리기)  q 종료
종료하면 상황별 요약(잡음 바닥값, 최대 전환 횟수, 사건 수, 처리 시간)을 출력한다.

아래 인자 기본값은 측정을 시작하기 위한 값일 뿐 Q-11 확정값이 아니다(2026-10-10).
"""
import argparse
import csv
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[1]
for path in sorted((ROOT / "src").glob("rescue_*")):
    sys.path.insert(0, str(path))

from rescue_perception import SourceBinding  # noqa: E402
from rescue_perception.disaster_sign import DisasterSignDetector, DisasterSignPolicy  # noqa: E402

LABELS = {ord("1"): "still", ord("2"): "shake", ord("3"): "light", ord("4"): "pass", ord("5"): "cover"}
FIELDS = ("stamp_ns", "label", "status", "reason", "dx", "dy", "reversals", "condition", "event",
          "textured_ratio", "matched_ratio", "proc_ms")


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", type=int, default=0, help="OpenCV 카메라 번호")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--stride", type=int, default=8, help="격자 1칸 픽셀")
    ap.add_argument("--blocks", type=int, nargs=2, default=(4, 4), metavar=("X", "Y"))
    ap.add_argument("--search", type=int, default=3, help="프레임당 찾을 최대 이동(칸)")
    ap.add_argument("--texture-min", type=float, default=4.0)
    ap.add_argument("--match-min", type=float, default=0.6)
    ap.add_argument("--valid-ratio", type=float, default=0.5)
    ap.add_argument("--agree-tol", type=float, default=1.0)
    ap.add_argument("--shift-min", type=float, default=1.0, help="방향을 기록할 최소 이동(칸)")
    ap.add_argument("--shakes", type=int, default=4, help="창 안 방향 전환 횟수 N")
    ap.add_argument("--window-s", type=float, default=3.0)
    ap.add_argument("--hold-s", type=float, default=0.2)
    ap.add_argument("--release-s", type=float, default=2.0)
    ap.add_argument("--max-gap-s", type=float, default=0.5)
    ap.add_argument("--csv", type=Path, help="프레임별 기록을 남길 CSV 경로")
    ap.add_argument("--no-window", action="store_true", help="화면 없이 콘솔만 (라벨 키 사용 불가)")
    return ap.parse_args()


def policy_from(args):
    ns = lambda s: int(s * 1e9)
    return DisasterSignPolicy(sample_stride=args.stride, blocks_x=args.blocks[0], blocks_y=args.blocks[1],
                              search_cells=args.search, texture_min=args.texture_min, match_min=args.match_min,
                              valid_ratio_min=args.valid_ratio, agree_tol=args.agree_tol,
                              shift_min=args.shift_min, shakes_min=args.shakes, window_ns=ns(args.window_s),
                              hold_ns=ns(args.hold_s), release_ns=ns(args.release_s),
                              max_gap_ns=ns(args.max_gap_s))


def to_image(gray, stamp_ns):
    """OpenCV 회색조 배열을 sensor_msgs/Image 모양으로 감싼다."""
    h, w = gray.shape
    return NS(header=NS(stamp=NS(sec=stamp_ns // 10**9, nanosec=stamp_ns % 10**9), frame_id="cam_fixed/optical"),
              width=w, height=h, encoding="mono8", is_bigendian=0, step=w, data=gray.tobytes())


def summarize(rows):
    """상황별 요약: 잡음 바닥값(가만히일 때 이동 크기), 전환 횟수, 사건, 처리 시간."""
    by_label = {}
    for r in rows:
        by_label.setdefault(r["label"], []).append(r)
    print("\n=== 상황별 요약 ===")
    for label, rs in by_label.items():
        mags = sorted(math.hypot(r["dx"], r["dy"]) for r in rs if r["dx"] != "")
        procs = sorted(r["proc_ms"] for r in rs)
        p95 = lambda xs: xs[min(len(xs) - 1, int(len(xs) * 0.95))] if xs else float("nan")
        unknown = sum(r["status"] == "UNKNOWN" for r in rs)
        print(f"[{label}] 프레임 {len(rs)}, 사건 {sum(r['event'] for r in rs)}, 최대 전환 {max(r['reversals'] for r in rs)}, "
              f"이동 크기 p95 {p95(mags):.2f} / 최대 {mags[-1] if mags else float('nan'):.2f} 칸, "
              f"UNKNOWN {unknown}, 처리 p95 {p95(procs):.0f} ms")


def main():
    args = parse_args()
    try:
        import cv2
    except ImportError:
        sys.exit("OpenCV(cv2)가 필요합니다: pip install opencv-python")
    detector = DisasterSignDetector(
        SourceBinding(None, "cam_fixed", str(uuid4()), "probe-host-clock", "DRIVER_RECEIVE", "UNKNOWN",
                      "uncalibrated", "UNKNOWN", None),
        policy_from(args))
    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        sys.exit(f"카메라 {args.camera} 를 열 수 없습니다 (macOS 는 터미널 앱의 카메라 권한 확인)")
    writer = None
    if args.csv:
        handle = args.csv.open("w", newline="")
        writer = csv.DictWriter(handle, FIELDS)
        writer.writeheader()
    label, rows, events = "still", [], 0
    print(f"정책: {detector.policy}")
    try:
        while True:
            ok, frame = cap.read()
            stamp = time.time_ns()  # 드라이버에서 받은 시각(촬영 시각 아님)
            if not ok:
                print("프레임을 읽지 못했습니다 (카메라 끊김) — 종료")
                break
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            t0 = time.perf_counter()
            result = detector.update(to_image(gray, stamp))
            proc_ms = (time.perf_counter() - t0) * 1000
            dx, dy = result.shift if result.shift is not None else ("", "")
            row = dict(stamp_ns=stamp, label=label, status=result.status, reason=result.reason or "",
                       dx=dx, dy=dy, reversals=result.reversals, condition=int(result.condition),
                       event=int(result.event is not None), textured_ratio=result.textured_ratio,
                       matched_ratio=result.matched_ratio, proc_ms=round(proc_ms, 1))
            rows.append(row)
            if writer:
                writer.writerow(row)
            if result.event is not None:
                events += 1
                print(f"*** DISASTER_SIGN #{events}  [{label}]  {result.event.detail}")
            if args.no_window:
                continue
            color = (0, 0, 255) if result.condition else (0, 200, 0)
            lines = (f"[{label}] {result.status} {result.reason or ''}",
                     f"shift {result.shift}  reversals {result.reversals}/{args.shakes}",
                     f"textured {result.textured_ratio}  matched {result.matched_ratio}",
                     f"proc {proc_ms:.0f} ms  events {events}")
            for i, text in enumerate(lines):
                cv2.putText(frame, text, (10, 25 + 25 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cv2.imshow("webcam quake probe (1-5 label, q quit)", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key in LABELS:
                label = LABELS[key]
                print(f"-- 상황: {label}")
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        if not args.no_window:
            cv2.destroyAllWindows()
        if writer:
            handle.close()
        summarize(rows)


if __name__ == "__main__":
    main()
