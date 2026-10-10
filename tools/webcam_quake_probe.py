#!/usr/bin/env python3
"""웹캠으로 지진 판정(흔들림 횟수)을 실시간 확인하고 Q-11 값을 재는 도구 (E3 예비 측정).

판정은 rescue_perception.disaster_sign 을 그대로 쓰고, 이 도구는 OpenCV 로 프레임을
읽어 넣어 주기만 한다. 로봇·ROS 를 쓰지 않는다.

사용:
    python3 tools/webcam_quake_probe.py --list                 # 연결된 카메라 번호 찾기
    python3 tools/webcam_quake_probe.py --camera 1 --csv quake_probe.csv

창에서 숫자 키로 지금 하는 상황을 표시하면 CSV 의 label 칸에 남는다. CSV 앞 두 칸은
실제 시각(time, 이 PC 기준)과 측정 시작부터 지난 초(elapsed_s)다.
    1 still(가만히)  2 shake(흔들기)  3 light(조명 켜기·끄기)  4 pass(앞 지나가기)
    5 cover(렌즈 가리기)  q 종료
종료하면 상황별 요약(잡음 바닥값, 최대 전환 횟수, 사건 수, 처리 시간)을 출력한다.

아래 인자 기본값은 측정을 시작하기 위한 값일 뿐 Q-11 확정값이 아니다(2026-10-10).
"""
import argparse
import csv
from datetime import datetime
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
FIELDS = ("time", "elapsed_s", "stamp_ns", "label", "status", "reason", "dx", "dy", "reversals", "condition", "event",
          "textured_ratio", "matched_ratio", "proc_ms")


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", type=int, default=0, help="OpenCV 카메라 번호 (--list 로 확인)")
    ap.add_argument("--list", action="store_true", help="0~4번 카메라를 열어 보고 해상도를 출력한 뒤 종료")
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
    ap.add_argument("--scene-match-min", type=float, default=0.5, help="기준 장면과 같은 장면으로 볼 ZNCC")
    ap.add_argument("--shakes", type=int, default=4, help="창 안 방향 전환 횟수 N")
    ap.add_argument("--window-s", type=float, default=3.0)
    ap.add_argument("--hold-s", type=float, default=0.2)
    ap.add_argument("--release-s", type=float, default=2.0)
    ap.add_argument("--max-gap-s", type=float, default=0.5)
    ap.add_argument("--csv", type=Path, help="프레임별 기록을 남길 CSV 경로")
    ap.add_argument("--record", type=Path, help="웹캠 원본 영상을 이 파일(.mp4)로 녹화")
    ap.add_argument("--video", type=Path, help="웹캠 대신 녹화한 영상을 재생해 판정(시각은 프레임 번호 / fps)")
    ap.add_argument("--no-window", action="store_true", help="화면 없이 콘솔만 (라벨 키 사용 불가)")
    return ap.parse_args()


def policy_from(args):
    ns = lambda s: int(s * 1e9)
    return DisasterSignPolicy(sample_stride=args.stride, blocks_x=args.blocks[0], blocks_y=args.blocks[1],
                              search_cells=args.search, texture_min=args.texture_min, match_min=args.match_min,
                              valid_ratio_min=args.valid_ratio, agree_tol=args.agree_tol,
                              shift_min=args.shift_min, scene_match_min=args.scene_match_min,
                              shakes_min=args.shakes, window_ns=ns(args.window_s),
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
        p95 = lambda xs: xs[min(len(xs) - 1, int(len(xs) * 0.95))] if xs else float("nan")
        unknown = sum(r["status"] == "UNKNOWN" for r in rs)
        lost = sum(r["status"] == "STREAM_LOST" for r in rs)
        procs = sorted(r["proc_ms"] for r in rs if r["status"] != "STREAM_LOST")
        print(f"[{label}] 프레임 {len(rs)}, 사건 {sum(r['event'] for r in rs)}, 최대 전환 {max(r['reversals'] for r in rs)}, "
              f"이동 크기 p95 {p95(mags):.2f} / 최대 {mags[-1] if mags else float('nan'):.2f} 칸, "
              f"UNKNOWN {unknown}, 카메라 끊김 {lost}, 처리 p95 {p95(procs):.0f} ms")


WINDOW = "webcam quake probe"
KEY_HELP = (("1", "still"), ("2", "shake"), ("3", "light"), ("4", "pass"), ("5", "cover"))
# OpenCV 기본 글꼴은 영문만 그린다. 색은 BGR.
GREEN, RED, GRAY, ORANGE, YELLOW, WHITE, BLACK = ((40, 160, 40), (40, 40, 220), (110, 110, 110),
                                                 (0, 140, 255), (0, 220, 255), (255, 255, 255), (0, 0, 0))


def _bar(cv2, frame, y0, y1, color):
    cv2.rectangle(frame, (0, y0), (frame.shape[1], y1), color, -1)


def draw_overlay(cv2, frame, label, result, args, proc_ms, events, banner):
    """맨 위: 지금 상황(라벨)과 판정 상태를 크게. 가운데: 전환 횟수 막대. 맨 아래: 키 안내."""
    h, w = frame.shape[:2]
    if result is None:
        state, color = "NO CAMERA", ORANGE
        banner = banner or "reconnecting camera..."
    elif result.status == "UNKNOWN":
        state, color = f"UNKNOWN ({result.reason})", GRAY
    elif result.condition:
        state, color = "SHAKING", RED
    else:
        state, color = "NORMAL", GREEN
    _bar(cv2, frame, 0, 60, color)
    cv2.putText(frame, f"MODE: {label.upper()}", (12, 42), cv2.FONT_HERSHEY_SIMPLEX, 1.1, WHITE, 3)
    size = cv2.getTextSize(state, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)[0]
    cv2.putText(frame, state, (w - size[0] - 12, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, WHITE, 2)
    if result is not None:
        # 방향 전환 횟수 / 기준 N 막대
        filled = min(1.0, result.reversals / args.shakes)
        cv2.rectangle(frame, (12, 72), (312, 92), WHITE, 2)
        cv2.rectangle(frame, (14, 74), (14 + int(296 * filled), 90), RED if filled >= 1 else YELLOW, -1)
        cv2.putText(frame, f"reversals {result.reversals}/{args.shakes}", (322, 89),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, WHITE, 2)
        info = (f"shift {result.shift}  reason {result.reason or '-'}",
                f"textured {result.textured_ratio}  matched {result.matched_ratio}",
                f"proc {proc_ms:.0f} ms   events {events}")
        _bar(cv2, frame, h - 120, h - 36, (50, 50, 50))
        for i, text in enumerate(info):
            cv2.putText(frame, text, (12, h - 96 + 24 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, WHITE, 1)
    if banner:
        # 사건이 난 직후 2초 동안 화면 가운데에 크게
        size = cv2.getTextSize(banner, cv2.FONT_HERSHEY_SIMPLEX, 1.4, 4)[0]
        x, y = (w - size[0]) // 2, h // 2
        fill = color if result is None else RED
        cv2.rectangle(frame, (x - 16, y - size[1] - 16), (x + size[0] + 16, y + 16), fill, -1)
        cv2.putText(frame, banner, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.4, WHITE, 4)
    _bar(cv2, frame, h - 36, h, BLACK)
    x = 12
    for key, name in KEY_HELP + (("q", "quit"),):
        text = f"[{key}] {name}"
        cv2.putText(frame, text, (x, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, YELLOW if name == label else WHITE,
                    2 if name == label else 1)
        x += cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0][0] + 18
    cv2.imshow(WINDOW, frame)


def handle_key(cv2, key, label):
    """q 면 None, 숫자 키면 새 라벨, 그 밖에는 그대로."""
    if key == ord("q"):
        return None
    if key in LABELS and LABELS[key] != label:
        print(f"-- 상황: {LABELS[key]}")
        return LABELS[key]
    return label


def clock(stamp_ns, started_ns):
    """CSV 앞 두 칸: 실제 시각(이 PC 기준, 밀리초까지)과 측정 시작부터 지난 초."""
    return dict(time=datetime.fromtimestamp(stamp_ns / 1e9).strftime("%H:%M:%S.%f")[:-3],
                elapsed_s=round((stamp_ns - started_ns) / 1e9, 3))


_RECORD_SIZE = {}


def recorder_size(recorder, frame):
    """녹화 파일은 처음 프레임 크기로 고정된다. 크기가 다른 프레임(카메라 켜질 때 등)은 건너뛴다."""
    return _RECORD_SIZE.setdefault(id(recorder), (frame.shape[1], frame.shape[0]))


def open_camera(cv2, args):
    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    return cap


def list_cameras(cv2):
    """번호마다 열어 보고 프레임 크기를 출력한다. 노트북 내장 카메라가 보통 0번이다."""
    for index in range(5):
        cap = cv2.VideoCapture(index)
        ok, frame = cap.read() if cap.isOpened() else (False, None)
        print(f"카메라 {index}: " + (f"열림 {frame.shape[1]}x{frame.shape[0]}" if ok else "없음"))
        cap.release()


def main():
    args = parse_args()
    try:
        import cv2
    except ImportError:
        sys.exit("OpenCV(cv2)가 필요합니다: pip install opencv-python")
    if args.list:
        list_cameras(cv2)
        return
    detector = DisasterSignDetector(
        SourceBinding(None, "cam_fixed", str(uuid4()), "probe-host-clock", "DRIVER_RECEIVE", "UNKNOWN",
                      "uncalibrated", "UNKNOWN", None),
        policy_from(args))
    cap = cv2.VideoCapture(str(args.video)) if args.video else open_camera(cv2, args)
    if args.video and not cap.isOpened():
        sys.exit(f"영상 {args.video} 를 열 수 없습니다")
    if not cap.isOpened():
        sys.exit(f"카메라 {args.camera} 를 열 수 없습니다 (macOS 는 터미널 앱의 카메라 권한 확인)")
    writer = None
    if args.csv:
        handle = args.csv.open("w", newline="")
        writer = csv.DictWriter(handle, FIELDS)
        writer.writeheader()
    label, rows, events, last_reversals, lost_since = "still", [], 0, 0, None
    last_frame, banner, banner_until = None, "", 0
    started = time.time_ns()
    video_fps = (cap.get(cv2.CAP_PROP_FPS) or 25.0) if args.video else None
    frame_index, recorder = 0, None
    print(f"정책: {detector.policy}")
    try:
        while True:
            ok, frame = cap.read()
            stamp = time.time_ns()  # 드라이버에서 받은 시각(촬영 시각 아님)
            if args.video:
                if not ok:
                    print("영상 끝")
                    break
                stamp = started + int(frame_index * 1e9 / video_fps)  # 녹화 영상은 프레임 번호로 시각을 만든다
                frame_index += 1
            if ok and args.record:
                if recorder is None:
                    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                    recorder = cv2.VideoWriter(str(args.record), fourcc, cap.get(cv2.CAP_PROP_FPS) or 25.0,
                                               (frame.shape[1], frame.shape[0]))
                if (frame.shape[1], frame.shape[0]) == recorder_size(recorder, frame):
                    recorder.write(frame)
            if not ok:
                # 끊김은 종료하지 않고 기록한 뒤 다시 연결한다. 판정기는 다음 프레임에서
                # 간격이 max_gap 을 넘은 것을 보고 횟수·구간을 스스로 초기화한다.
                if lost_since is None:
                    lost_since = stamp
                    print("프레임을 읽지 못했습니다 (카메라 끊김) — 다시 연결 시도 중 (q 로 종료)")
                row = dict(**clock(stamp, started), stamp_ns=stamp, label=label, status="STREAM_LOST", reason="", dx="", dy="",
                           reversals=last_reversals, condition=0, event=0, textured_ratio="",
                           matched_ratio="", proc_ms=0.0)
                rows.append(row)
                if writer:
                    writer.writerow(row)
                if args.no_window:
                    time.sleep(0.5)
                else:
                    if last_frame is not None:
                        draw_overlay(cv2, last_frame.copy(), label, None, args, 0, events, "")
                    label = handle_key(cv2, cv2.waitKey(500) & 0xFF, label)
                    if label is None:
                        break
                cap.release()
                cap = open_camera(cv2, args)
                continue
            if lost_since is not None:
                print(f"카메라 다시 연결됨 ({(stamp - lost_since) / 1e9:.1f}초 끊김)")
                lost_since = None
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            t0 = time.perf_counter()
            result = detector.update(to_image(gray, stamp))
            proc_ms = (time.perf_counter() - t0) * 1000
            dx, dy = result.shift if result.shift is not None else ("", "")
            row = dict(**clock(stamp, started), stamp_ns=stamp, label=label, status=result.status, reason=result.reason or "",
                       dx=dx, dy=dy, reversals=result.reversals, condition=int(result.condition),
                       event=int(result.event is not None), textured_ratio=result.textured_ratio,
                       matched_ratio=result.matched_ratio, proc_ms=round(proc_ms, 1))
            rows.append(row)
            last_reversals = result.reversals
            if writer:
                writer.writerow(row)
            if result.event is not None:
                events += 1
                banner, banner_until = f"DISASTER_SIGN #{events}", stamp + 2 * 10**9
                print(f"*** DISASTER_SIGN #{events}  [{label}]  {result.event.detail}")
            if args.no_window:
                continue
            last_frame = frame.copy()
            draw_overlay(cv2, frame, label, result, args, proc_ms, events, banner if stamp < banner_until else "")
            label = handle_key(cv2, cv2.waitKey(1) & 0xFF, label)
            if label is None:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        if recorder is not None:
            recorder.release()
            print(f"녹화 저장: {args.record.resolve()}")
        if not args.no_window:
            cv2.destroyAllWindows()
        if writer:
            handle.close()
        summarize(rows)
        if args.csv:
            print(f"프레임별 기록 저장: {args.csv.resolve()}")


if __name__ == "__main__":
    main()
