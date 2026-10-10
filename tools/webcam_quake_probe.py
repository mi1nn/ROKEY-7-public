#!/usr/bin/env python3
"""웹캠으로 지진 판정(흔들림 횟수)을 실시간 확인하고 Q-11 값을 재는 도구 (E3 예비 측정).

판정은 rescue_perception.disaster_sign 을 그대로 쓰고, 이 도구는 OpenCV 로 프레임을
읽어 넣고 결과를 대시보드로 보여 주기만 한다. 로봇·ROS 를 쓰지 않는다.

사용:
    python3 tools/webcam_quake_probe.py --list               # 연결된 카메라 번호 찾기
    python3 tools/webcam_quake_probe.py --camera 0 --record  # 측정 + 화면 녹화
    python3 tools/webcam_quake_probe.py --video .artifacts/webcam_quake/<측정>/raw.mp4   # 녹화 재생

창에서 숫자 키로 지금 하는 상황을 표시하면 기록의 label 칸에 남는다.
    1 가만히(still)  2 흔들기(shake)  3 조명(light)  4 지나가기(pass)  5 가리기(cover)  q 종료

결과는 측정마다 .artifacts/webcam_quake/<날짜-시각>/ 에 저장한다(.gitignore 대상, 커밋 안 함).
    probe.csv  프레임별 판정 기록(앞 두 칸은 실제 시각과 경과 초)
    meta.json  실행 인자·판정 정책·git SHA·상황별 요약
    view.mp4   대시보드 화면 그대로 (--record)
    raw.mp4    웹캠 원본, --video 로 다시 판정할 때 쓴다 (--record)

인자 기본값은 측정을 시작하기 위한 값일 뿐 Q-11 확정값이 아니다(2026-10-10).
한글 표시에 Pillow 와 한글 글꼴(macOS 기본 Apple SD Gothic Neo)을 쓴다. 없으면 영문 글꼴로 그린다.
"""
import argparse
from collections import deque
import csv
from dataclasses import asdict
from datetime import datetime
import json
import math
from pathlib import Path
import subprocess
import sys
import time
from types import SimpleNamespace as NS
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[1]
for path in sorted((ROOT / "src").glob("rescue_*")):
    sys.path.insert(0, str(path))

from rescue_perception import SourceBinding  # noqa: E402
from rescue_perception.disaster_sign import DisasterSignDetector, DisasterSignPolicy  # noqa: E402

OUT_ROOT = ROOT / ".artifacts" / "webcam_quake"
LABELS = {ord("1"): "still", ord("2"): "shake", ord("3"): "light", ord("4"): "pass", ord("5"): "cover"}
LABEL_KO = {"still": "가만히", "shake": "흔들기", "light": "조명", "pass": "지나가기", "cover": "가리기"}
REASON_KO = {"FIRST_FRAME": "첫 프레임", "OCCLUDED": "무늬 부족 (어둡거나 렌즈 가림)",
             "MATCH_FAILED": "직전 화면과 맞지 않음", "SCENE_CHANGED": "원래 장면이 가려짐",
             "PARTIAL_MOTION": "화면 일부만 움직임 (사람 등)", "BELOW_NOISE": "움직임 없음", None: "흔들림 기록 중"}
FIELDS = ("time", "elapsed_s", "stamp_ns", "label", "status", "reason", "dx", "dy", "reversals", "condition",
          "event", "textured_ratio", "matched_ratio", "visible_ratio", "proc_ms")
FONT_PATHS = ("/System/Library/Fonts/AppleSDGothicNeo.ttc", "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
              "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")

# 색은 OpenCV 순서(BGR). 어두운 배경의 대시보드.
BG, CARD, LINE = (32, 28, 25), (48, 43, 39), (78, 72, 66)
TEXT, MUTED = (240, 240, 240), (165, 160, 155)
GREEN, RED, GRAY, ORANGE, YELLOW, BLUE = ((80, 175, 76), (53, 57, 229), (120, 120, 120), (0, 140, 245),
                                          (40, 202, 255), (245, 165, 66))
PANEL_W, VIEW_H = 380, 480


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", type=int, default=0, help="OpenCV 카메라 번호 (--list 로 확인)")
    ap.add_argument("--list", action="store_true", help="0~4번 카메라를 열어 보고 해상도를 출력한 뒤 종료")
    ap.add_argument("--video", type=Path, help="웹캠 대신 녹화한 원본 영상(raw.mp4)을 판정(시각은 프레임 번호 / fps)")
    ap.add_argument("--record", action="store_true", help="대시보드 화면(view.mp4)과 원본(raw.mp4)을 녹화")
    ap.add_argument("--out", type=Path, help=f"결과 폴더 (기본: {OUT_ROOT.relative_to(ROOT)}/<날짜-시각>)")
    ap.add_argument("--no-window", action="store_true", help="화면 없이 콘솔만 (라벨 키 사용 불가)")
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
    ap.add_argument("--scene-match-min", type=float, default=0.8, help="블록이 기준 장면과 같다고 볼 ZNCC")
    ap.add_argument("--rebase-s", type=float, default=10.0, help="바뀐 화면을 새 기준 장면으로 삼을 안정 시간(초)")
    ap.add_argument("--shakes", type=int, default=4, help="창 안 방향 전환 횟수 N")
    ap.add_argument("--window-s", type=float, default=3.0)
    ap.add_argument("--hold-s", type=float, default=0.2)
    ap.add_argument("--release-s", type=float, default=2.0)
    ap.add_argument("--max-gap-s", type=float, default=0.5)
    return ap.parse_args()


def policy_from(args):
    ns = lambda s: int(s * 1e9)
    return DisasterSignPolicy(sample_stride=args.stride, blocks_x=args.blocks[0], blocks_y=args.blocks[1],
                              search_cells=args.search, texture_min=args.texture_min, match_min=args.match_min,
                              valid_ratio_min=args.valid_ratio, agree_tol=args.agree_tol,
                              shift_min=args.shift_min, scene_match_min=args.scene_match_min,
                              rebase_ns=ns(args.rebase_s), shakes_min=args.shakes, window_ns=ns(args.window_s),
                              hold_ns=ns(args.hold_s), release_ns=ns(args.release_s),
                              max_gap_ns=ns(args.max_gap_s))


def to_image(gray, stamp_ns):
    """OpenCV 회색조 배열을 sensor_msgs/Image 모양으로 감싼다."""
    h, w = gray.shape
    return NS(header=NS(stamp=NS(sec=stamp_ns // 10**9, nanosec=stamp_ns % 10**9), frame_id="cam_fixed/optical"),
              width=w, height=h, encoding="mono8", is_bigendian=0, step=w, data=gray.tobytes())


def clock(stamp_ns, started_ns):
    """기록 앞 두 칸: 실제 시각(이 PC 기준, 밀리초까지)과 측정 시작부터 지난 초."""
    return dict(time=datetime.fromtimestamp(stamp_ns / 1e9).strftime("%H:%M:%S.%f")[:-3],
                elapsed_s=round((stamp_ns - started_ns) / 1e9, 3))


def git_sha():
    try:
        return subprocess.run(["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"], capture_output=True,
                              text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def summarize(rows):
    """상황별 요약: 잡음 바닥값(가만히일 때 이동 크기), 전환 횟수, 사건, UNKNOWN, 처리 시간."""
    by_label, out = {}, {}
    for r in rows:
        by_label.setdefault(r["label"], []).append(r)
    print("\n=== 상황별 요약 ===")
    p95 = lambda xs: xs[min(len(xs) - 1, int(len(xs) * 0.95))] if xs else float("nan")
    for label, rs in by_label.items():
        mags = sorted(math.hypot(r["dx"], r["dy"]) for r in rs if r["dx"] != "")
        procs = sorted(r["proc_ms"] for r in rs if r["status"] != "STREAM_LOST")
        item = dict(frames=len(rs), events=sum(r["event"] for r in rs), max_reversals=max(r["reversals"] for r in rs),
                    shift_p95=p95(mags), shift_max=mags[-1] if mags else float("nan"),
                    unknown=sum(r["status"] == "UNKNOWN" for r in rs),
                    stream_lost=sum(r["status"] == "STREAM_LOST" for r in rs), proc_ms_p95=p95(procs))
        out[label] = item
        print(f"[{label}] 프레임 {item['frames']}, 사건 {item['events']}, 최대 전환 {item['max_reversals']}, "
              f"이동 크기 p95 {item['shift_p95']:.2f} / 최대 {item['shift_max']:.2f} 칸, "
              f"UNKNOWN {item['unknown']}, 카메라 끊김 {item['stream_lost']}, 처리 p95 {item['proc_ms_p95']:.0f} ms")
    return out


def load_fonts():
    try:
        from PIL import ImageFont
    except ImportError:
        return None
    for path in FONT_PATHS:
        try:
            make = lambda size, bold=False, path=path: ImageFont.truetype(
                path, size, index=(6 if bold else 0) if path.endswith(".ttc") else 0)
            make(12)
            return make
        except OSError:
            continue
    return None


class Dashboard:
    """왼쪽은 카메라 화면(블록 격자·이동 화살표), 오른쪽은 상태 패널."""

    def __init__(self, cv2, args):
        self.cv2, self.args = cv2, args
        self.make_font, self._fonts = load_fonts(), {}
        self.history = deque(maxlen=150)   # (경과 초, 이동 크기 또는 None, 방향 전환 여부)
        self.events = deque(maxlen=3)      # (시각, 상황)
        self.banner_until, self.event_count, self._last_rev = 0, 0, 0

    def font(self, size, bold=False):
        key = (size, bold)
        if key not in self._fonts:
            self._fonts[key] = self.make_font(size, bold)
        return self._fonts[key]

    def note(self, stamp, elapsed, result, label):
        """프레임 결과를 그래프·사건 기록에 쌓는다."""
        mag = math.hypot(*result.shift) if result.shift is not None else None
        reversed_now = result.reversals > self._last_rev
        self._last_rev = result.reversals
        self.history.append((elapsed, mag, reversed_now))
        if result.event is not None:
            self.event_count += 1
            self.banner_until = stamp + 2 * 10**9
            self.events.appendleft((datetime.fromtimestamp(stamp / 1e9).strftime("%H:%M:%S"), label))

    def render(self, frame, label, result, stamp, proc_ms, fps):
        cv2, np = self.cv2, __import__("numpy")
        h0, w0 = frame.shape[:2]
        view = cv2.resize(frame, (round(w0 * VIEW_H / h0), VIEW_H)) if h0 != VIEW_H else frame.copy()
        vw = view.shape[1]
        canvas = np.full((VIEW_H, vw + PANEL_W, 3), BG, np.uint8)
        state, color, detail = self._state(result)
        texts = []
        self._draw_view(view, result, color)
        canvas[:, :vw] = view
        if stamp < self.banner_until:
            cv2.rectangle(canvas, (0, VIEW_H // 2 - 42), (vw, VIEW_H // 2 + 42), RED, -1)
            texts.append(((vw // 2, VIEW_H // 2), f"지진 징후 감지 #{self.event_count}", 38, TEXT, True, "mm"))
        self._draw_panel(canvas, vw, label, result, state, color, detail, proc_ms, fps, stamp, texts)
        self._draw_texts(canvas, texts)
        return canvas

    def _state(self, result):
        if result is None:
            return "카메라 끊김", ORANGE, "다시 연결하는 중"
        if result.status == "UNKNOWN":
            detail = REASON_KO.get(result.reason, result.reason)
            if result.reason == "SCENE_CHANGED" and result.visible_ratio is not None:
                detail += f" ({result.visible_ratio * 100:.0f}% 보임)"
            return "판단 보류", GRAY, detail
        if result.condition:
            return "흔들림 감지", RED, f"방향 전환 {result.reversals}회"
        return "정상", GREEN, REASON_KO.get(result.reason, result.reason)

    def _draw_view(self, view, result, color):
        cv2, a = self.cv2, self.args
        h, w = view.shape[:2]
        overlay = view.copy()
        for i in range(1, a.blocks[0]):      # 블록 격자, 가운데(사분면 경계)는 굵게
            x = w * i // a.blocks[0]
            cv2.line(overlay, (x, 0), (x, h), TEXT, 2 if i * 2 == a.blocks[0] else 1, cv2.LINE_AA)
        for j in range(1, a.blocks[1]):
            y = h * j // a.blocks[1]
            cv2.line(overlay, (0, y), (w, y), TEXT, 2 if j * 2 == a.blocks[1] else 1, cv2.LINE_AA)
        cv2.addWeighted(overlay, 0.25, view, 0.75, 0, view)
        if result is not None and result.shift is not None and math.hypot(*result.shift) > 0:
            scale = a.stride * 6 * VIEW_H / max(1, a.height)  # 화면에서 잘 보이게 키운 이동 화살표
            cx, cy = w // 2, h // 2
            end = (int(cx + result.shift[0] * scale), int(cy + result.shift[1] * scale))
            cv2.arrowedLine(view, (cx, cy), end, YELLOW, 4, cv2.LINE_AA, tipLength=0.3)
        cv2.rectangle(view, (0, 0), (w - 1, h - 1), color, 6)

    def _draw_panel(self, c, x0, label, result, state, color, detail, proc_ms, fps, stamp, texts):
        cv2, a = self.cv2, self.args
        W = x0 + PANEL_W
        # 머리
        texts.append(((x0 + 18, 16), "지진 감지 모니터", 20, TEXT, True, "la"))
        texts.append(((W - 18, 19), datetime.fromtimestamp(stamp / 1e9).strftime("%H:%M:%S"), 16, MUTED, False, "ra"))
        # 상태 카드
        cv2.rectangle(c, (x0 + 14, 52), (W - 14, 136), color, -1)
        texts.append(((x0 + 30, 60), state, 32, TEXT, True, "la"))
        texts.append(((x0 + 30, 106), detail or "", 15, TEXT, False, "la"))
        # 상황 칩
        texts.append(((x0 + 18, 148), "지금 상황 (숫자 키)", 13, MUTED, False, "la"))
        cw, gap, y = (PANEL_W - 28 - 4 * 6) // 5, 6, 168
        for i, key in enumerate(("still", "shake", "light", "pass", "cover")):
            xa = x0 + 14 + i * (cw + gap)
            on = key == label
            cv2.rectangle(c, (xa, y), (xa + cw, y + 30), YELLOW if on else CARD, -1)
            texts.append(((xa + cw // 2, y + 15), f"{i + 1} {LABEL_KO[key]}", 13, BG if on else TEXT, on, "mm"))
        # 방향 전환 막대
        rev = result.reversals if result is not None else 0
        texts.append(((x0 + 18, 212), f"방향 전환 (최근 {a.window_s:g}초)", 14, MUTED, False, "la"))
        texts.append(((W - 18, 210), f"{rev} / {a.shakes}", 18, TEXT, True, "ra"))
        self._bar(c, x0 + 18, 236, PANEL_W - 36, 12, min(1.0, rev / a.shakes), RED if rev >= a.shakes else YELLOW)
        # 비율 게이지(가는 선 = 기준 valid_ratio)
        gauges = (("무늬", result.textured_ratio if result else None),
                  ("매칭", result.matched_ratio if result else None),
                  ("장면 보임", result.visible_ratio if result else None))
        for i, (name, value) in enumerate(gauges):
            y = 264 + i * 22
            texts.append(((x0 + 18, y - 2), name, 13, MUTED, False, "la"))
            fill = 0.0 if value is None else value
            self._bar(c, x0 + 92, y + 2, PANEL_W - 150, 10, fill, GREEN if fill >= a.valid_ratio else GRAY)
            tx = x0 + 92 + int((PANEL_W - 150) * a.valid_ratio)
            cv2.line(c, (tx, y - 1), (tx, y + 15), TEXT, 1)
            texts.append(((W - 18, y - 2), "-" if value is None else f"{value * 100:.0f}%", 13, TEXT, False, "ra"))
        # 움직임 그래프
        gy, gh = 350, 62
        texts.append(((x0 + 18, gy - 18), "화면 이동 크기 (최근 5초, 빨간 점 = 방향 전환)", 12, MUTED, False, "la"))
        cv2.rectangle(c, (x0 + 14, gy), (W - 14, gy + gh), CARD, -1)
        self._graph(c, x0 + 14, gy, PANEL_W - 28, gh)
        # 사건 기록과 바닥글
        texts.append(((x0 + 18, 420), f"지진 징후 사건 {self.event_count}개", 14, TEXT, True, "la"))
        for i, (t, lab) in enumerate(self.events):
            texts.append(((x0 + 160 + i * 72, 421), t, 12, RED, False, "la"))
        texts.append(((x0 + 18, 452), f"{fps:4.1f} fps   처리 {proc_ms:3.0f} ms   q 종료", 13, MUTED, False, "la"))

    def _bar(self, c, x, y, w, h, fill, color):
        self.cv2.rectangle(c, (x, y), (x + w, y + h), LINE, -1)
        if fill > 0:
            self.cv2.rectangle(c, (x, y), (x + int(w * fill), y + h), color, -1)

    def _graph(self, c, x, y, w, h):
        cv2, a = self.cv2, self.args
        if not self.history:
            return
        t_end, span = self.history[-1][0], 5.0
        top = max(a.search * 1.5, a.shift_min * 2)
        to_xy = lambda t, m: (int(x + w * (1 - (t_end - t) / span)), int(y + h - 4 - (h - 8) * min(m, top) / top))
        ny = to_xy(t_end, a.shift_min)[1]
        for xx in range(x, x + w, 8):          # 점선 = shift_min(이보다 작으면 잡음)
            cv2.line(c, (xx, ny), (min(xx + 4, x + w), ny), MUTED, 1)
        prev = None
        for t, m, rev in self.history:
            if t_end - t > span:
                continue
            if m is None:
                prev = None
                continue
            p = to_xy(t, m)
            if prev is not None:
                cv2.line(c, prev, p, BLUE, 2, cv2.LINE_AA)
            if rev:
                cv2.circle(c, p, 4, RED, -1, cv2.LINE_AA)
            prev = p

    def _draw_texts(self, canvas, texts):
        if self.make_font is None:            # Pillow·한글 글꼴이 없으면 OpenCV 영문 글꼴
            for (x, y), text, size, color, bold, anchor in texts:
                text = text.encode("ascii", "replace").decode()
                scale = size / 30
                tw, th = self.cv2.getTextSize(text, self.cv2.FONT_HERSHEY_SIMPLEX, scale, 1)[0]
                x = x - tw if anchor[0] == "r" else (x - tw // 2 if anchor[0] == "m" else x)
                y = y + th // 2 if anchor[1] == "m" else y + th
                self.cv2.putText(canvas, text, (x, y), self.cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2 if bold else 1,
                                 self.cv2.LINE_AA)
            return
        from PIL import Image, ImageDraw
        img = Image.fromarray(canvas)         # BGR 배열 그대로: 색도 BGR 로 넘긴다
        draw = ImageDraw.Draw(img)
        for xy, text, size, color, bold, anchor in texts:
            draw.text(xy, text, font=self.font(size, bold), fill=color, anchor=anchor)
        canvas[:] = __import__("numpy").asarray(img)


def handle_key(key, label):
    """q 면 None, 숫자 키면 새 라벨, 그 밖에는 그대로."""
    if key == ord("q"):
        return None
    if key in LABELS and LABELS[key] != label:
        print(f"-- 상황: {LABELS[key]} ({LABEL_KO[LABELS[key]]})")
        return LABELS[key]
    return label


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


class Recorder:
    """처음 프레임 크기로 mp4 를 연다. 크기가 다른 프레임(카메라 켜질 때 등)은 건너뛴다."""

    def __init__(self, cv2, path, fps):
        self.cv2, self.path, self.fps, self.writer, self.size = cv2, path, fps, None, None

    def write(self, frame):
        size = (frame.shape[1], frame.shape[0])
        if self.writer is None:
            self.size = size
            self.writer = self.cv2.VideoWriter(str(self.path), self.cv2.VideoWriter_fourcc(*"mp4v"), self.fps, size)
        if size == self.size:
            self.writer.write(frame)

    def close(self):
        if self.writer is not None:
            self.writer.release()
            return True
        return False


def main():
    args = parse_args()
    try:
        import cv2
    except ImportError:
        sys.exit("OpenCV(cv2)가 필요합니다: pip install opencv-python")
    if args.list:
        list_cameras(cv2)
        return
    policy = policy_from(args)
    detector = DisasterSignDetector(
        SourceBinding(None, "cam_fixed", str(uuid4()), "probe-host-clock", "DRIVER_RECEIVE", "UNKNOWN",
                      "uncalibrated", "UNKNOWN", None), policy)
    cap = cv2.VideoCapture(str(args.video)) if args.video else open_camera(cv2, args)
    if not cap.isOpened():
        sys.exit(f"영상 {args.video} 를 열 수 없습니다" if args.video else
                 f"카메라 {args.camera} 를 열 수 없습니다 (macOS 는 터미널 앱의 카메라 권한 확인)")
    out = args.out or OUT_ROOT / (datetime.now().strftime("%Y%m%d-%H%M%S") + ("-replay" if args.video else ""))
    out.mkdir(parents=True, exist_ok=True)
    fps_hint = cap.get(cv2.CAP_PROP_FPS) or 25.0
    view_rec = Recorder(cv2, out / "view.mp4", fps_hint) if args.record else None
    raw_rec = Recorder(cv2, out / "raw.mp4", fps_hint) if args.record and not args.video else None
    meta = dict(started=datetime.now().isoformat(timespec="seconds"), git_sha=git_sha(),
                source=str(args.video) if args.video else f"camera {args.camera}",
                args={k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()}, policy=asdict(policy))
    print(f"정책: {policy}\n결과 폴더: {out}")
    dash = None if args.no_window else Dashboard(cv2, args)
    handle = (out / "probe.csv").open("w", newline="")
    writer = csv.DictWriter(handle, FIELDS)
    writer.writeheader()
    label, rows, last_reversals, lost_since = "still", [], 0, None
    last_frame, frame_index, fps, last_stamp = None, 0, 0.0, None
    started = time.time_ns()
    try:
        while True:
            ok, frame = cap.read()
            stamp = time.time_ns()  # 드라이버에서 받은 시각(촬영 시각 아님)
            if args.video:
                if not ok:
                    print("영상 끝")
                    break
                stamp = started + int(frame_index * 1e9 / fps_hint)  # 녹화 영상은 프레임 번호로 시각을 만든다
                frame_index += 1
            if not ok:
                # 끊김은 종료하지 않고 기록한 뒤 다시 연결한다. 판정기는 다음 프레임에서
                # 간격이 max_gap 을 넘은 것을 보고 횟수·구간을 스스로 초기화한다.
                if lost_since is None:
                    lost_since = stamp
                    print("프레임을 읽지 못했습니다 (카메라 끊김) — 다시 연결 시도 중 (q 로 종료)")
                row = dict(**clock(stamp, started), stamp_ns=stamp, label=label, status="STREAM_LOST", reason="",
                           dx="", dy="", reversals=last_reversals, condition=0, event=0, textured_ratio="",
                           matched_ratio="", visible_ratio="", proc_ms=0.0)
                rows.append(row)
                writer.writerow(row)
                if dash is None:
                    time.sleep(0.5)
                else:
                    if last_frame is not None:
                        canvas = dash.render(last_frame, label, None, stamp, 0, 0)
                        cv2.imshow("webcam quake probe", canvas)
                        if view_rec:
                            view_rec.write(canvas)
                    label = handle_key(cv2.waitKey(500) & 0xFF, label)
                    if label is None:
                        break
                cap.release()
                cap = open_camera(cv2, args)
                continue
            if lost_since is not None:
                print(f"카메라 다시 연결됨 ({(stamp - lost_since) / 1e9:.1f}초 끊김)")
                lost_since = None
            if raw_rec:
                raw_rec.write(frame)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            t0 = time.perf_counter()
            result = detector.update(to_image(gray, stamp))
            proc_ms = (time.perf_counter() - t0) * 1000
            if last_stamp is not None and stamp > last_stamp:
                fps = 0.9 * fps + 0.1 * (1e9 / (stamp - last_stamp)) if fps else 1e9 / (stamp - last_stamp)
            last_stamp = stamp
            dx, dy = result.shift if result.shift is not None else ("", "")
            row = dict(**clock(stamp, started), stamp_ns=stamp, label=label, status=result.status,
                       reason=result.reason or "", dx=dx, dy=dy, reversals=result.reversals,
                       condition=int(result.condition), event=int(result.event is not None),
                       textured_ratio=result.textured_ratio, matched_ratio=result.matched_ratio,
                       visible_ratio="" if result.visible_ratio is None else round(result.visible_ratio, 3),
                       proc_ms=round(proc_ms, 1))
            rows.append(row)
            writer.writerow(row)
            last_reversals = result.reversals
            if result.event is not None:
                print(f"*** DISASTER_SIGN [{row['time']}] [{label}]  {result.event.detail}")
            if dash is None:
                continue
            dash.note(stamp, row["elapsed_s"], result, label)
            last_frame = frame
            canvas = dash.render(frame, label, result, stamp, proc_ms, fps)
            if view_rec:
                view_rec.write(canvas)
            cv2.imshow("webcam quake probe", canvas)
            label = handle_key(cv2.waitKey(1) & 0xFF, label)
            if label is None:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        handle.close()
        if dash is not None:
            cv2.destroyAllWindows()
        meta["summary"] = summarize(rows)
        meta["ended"] = datetime.now().isoformat(timespec="seconds")
        (out / "meta.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2, default=str))
        saved = ["probe.csv", "meta.json"] + [r.path.name for r in (view_rec, raw_rec) if r and r.close()]
        print(f"\n저장: {out}\n  " + ", ".join(saved))


if __name__ == "__main__":
    main()
