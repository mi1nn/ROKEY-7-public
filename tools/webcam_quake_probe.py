#!/usr/bin/env python3
"""웹캠으로 지진 판정(흔들림 횟수)을 실시간 확인하고 Q-11 값을 재는 도구 (E3 예비 측정).

판정은 rescue_perception.disaster_sign 을 그대로 쓰고, 이 도구는 OpenCV 로 프레임을
읽어 넣고 결과를 대시보드로 보여 주기만 한다. 로봇·ROS 를 쓰지 않는다.

사용:
    python3 tools/webcam_quake_probe.py --list               # 연결된 카메라 번호 찾기
    python3 tools/webcam_quake_probe.py --camera 0 --record  # 측정 + 화면 녹화
    python3 tools/webcam_quake_probe.py --video 영상.mp4      # 웹캠 대신 영상 파일로

창에서 숫자 키로 지금 하는 상황을 표시하면 기록의 label 칸에 남는다.
    1 가만히(still)  2 흔들기(shake)  3 조명(light)  4 지나가기(pass)  5 가리기(cover)  q 종료

결과는 측정마다 .artifacts/webcam_quake/<날짜-시각>/ 에 저장한다(.gitignore 대상, 커밋 안 함).
    probe.csv  프레임별 판정 기록(앞 두 칸은 실제 시각과 경과 초)
    meta.json  실행 인자·판정 정책·git SHA·상황별 요약
    view.mp4   대시보드 화면 그대로 (--record)

인자 기본값은 측정을 시작하기 위한 값일 뿐 Q-11 확정값이 아니다(2026-10-10).
화면은 Pillow 와 한글 글꼴(macOS 기본 Apple SD Gothic Neo)로 그린다. 없으면 --no-window 로만 돈다.
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
             "QUADRANT_COVERED": "한 구역이 완전히 가려짐",
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
PANEL_W, VIEW_H = 400, 480


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", type=int, default=0, help="OpenCV 카메라 번호 (--list 로 확인)")
    ap.add_argument("--list", action="store_true", help="0~4번 카메라를 열어 보고 해상도를 출력한 뒤 종료")
    ap.add_argument("--video", type=Path, help="웹캠 대신 영상 파일을 판정(시각은 프레임 번호 / fps)")
    ap.add_argument("--record", action="store_true", help="대시보드 화면을 view.mp4 로 녹화")
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
    ap.add_argument("--scene-match-min", type=float, default=0.6, help="블록이 기준 장면과 같다고 볼 ZNCC")
    ap.add_argument("--visible-min", type=float, default=0.3, help="원래 장면이 이 비율 미만으로 보이면 판단 보류")
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
                              visible_min=args.visible_min,
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


def load_font_maker():
    """한글 글꼴로 PIL 글꼴을 만드는 함수. Pillow 나 한글 글꼴이 없으면 None."""
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
    """왼쪽은 카메라 화면(블록 격자·이동 화살표·상황 표시), 오른쪽은 상태 패널.

    도형과 글자는 Pillow 로 그린다(둥근 모서리·반투명·한글). 배열은 OpenCV 의 BGR 순서
    그대로 두고 색도 BGR 로 넘긴다.
    """

    def __init__(self, cv2, args, make_font, replay):
        self.cv2, self.args, self.make_font, self.replay = cv2, args, make_font, replay
        self._fonts = {}
        self.history = deque(maxlen=200)   # (경과 초, 이동 크기 또는 None, 방향 전환 여부)
        self.events = deque(maxlen=3)      # 사건 시각
        self.banner_until, self.event_count, self._last_rev = 0, 0, 0

    def font(self, size, bold=False):
        if (size, bold) not in self._fonts:
            self._fonts[(size, bold)] = self.make_font(size, bold)
        return self._fonts[(size, bold)]

    def note(self, stamp, elapsed, result):
        """프레임 결과를 그래프·사건 기록에 쌓는다."""
        mag = math.hypot(*result.shift) if result.shift is not None else None
        self.history.append((elapsed, mag, result.reversals > self._last_rev))
        self._last_rev = result.reversals
        if result.event is not None:
            self.event_count += 1
            self.banner_until = stamp + 2 * 10**9
            self.events.appendleft(datetime.fromtimestamp(stamp / 1e9).strftime("%H:%M:%S"))

    def render(self, frame, label, result, stamp, proc_ms, fps):
        from PIL import Image, ImageDraw
        import numpy as np
        cv2 = self.cv2
        h0, w0 = frame.shape[:2]
        view = cv2.resize(frame, (round(w0 * VIEW_H / h0), VIEW_H)) if h0 != VIEW_H else frame.copy()
        vw = view.shape[1]
        state, color, detail, mark = self._state(result)
        self._draw_view(view, result)
        canvas = np.full((VIEW_H, vw + PANEL_W, 3), BG, np.uint8)
        canvas[:, :vw] = view
        img = Image.fromarray(canvas).convert("RGBA")
        layer = Image.new("RGBA", img.size, (0, 0, 0, 0))   # 반투명 도형
        d = ImageDraw.Draw(layer)
        self._view_overlay(d, vw, label, color, stamp)
        self._panel(d, vw, label, result, state, color, detail, mark, proc_ms, fps, stamp)
        img = Image.alpha_composite(img, layer)
        return np.asarray(img.convert("RGB")).copy()

    def _state(self, result):
        if result is None:
            return "카메라 끊김", ORANGE, "다시 연결하는 중", "!"
        if result.status == "UNKNOWN":
            detail = REASON_KO.get(result.reason, result.reason)
            if result.reason in ("SCENE_CHANGED", "QUADRANT_COVERED") and result.visible_ratio is not None:
                detail += f" · {result.visible_ratio * 100:.0f}% 보임"
            return "판단 보류", GRAY, detail, "?"
        if result.condition:
            return "흔들림 감지", RED, f"방향 전환 {result.reversals}회 (기준 {self.args.shakes}회)", "!"
        return "정상", GREEN, REASON_KO.get(result.reason, result.reason), "OK"

    def _draw_view(self, view, result):
        """카메라 화면 위: 블록 격자(사분면 경계는 굵게)와 화면 이동 화살표."""
        cv2, a = self.cv2, self.args
        h, w = view.shape[:2]
        overlay = view.copy()
        for i in range(1, a.blocks[0]):
            x = w * i // a.blocks[0]
            cv2.line(overlay, (x, 0), (x, h), TEXT, 2 if i * 2 == a.blocks[0] else 1, cv2.LINE_AA)
        for j in range(1, a.blocks[1]):
            y = h * j // a.blocks[1]
            cv2.line(overlay, (0, y), (w, y), TEXT, 2 if j * 2 == a.blocks[1] else 1, cv2.LINE_AA)
        cv2.addWeighted(overlay, 0.22, view, 0.78, 0, view)
        if result is not None and result.shift is not None and math.hypot(*result.shift) > 0:
            scale = a.stride * 6 * VIEW_H / max(1, a.height)  # 화면에서 잘 보이게 키운 이동 화살표
            cx, cy = w // 2, h // 2
            end = (int(cx + result.shift[0] * scale), int(cy + result.shift[1] * scale))
            cv2.arrowedLine(view, (cx, cy), end, BG, 9, cv2.LINE_AA, tipLength=0.3)
            cv2.arrowedLine(view, (cx, cy), end, YELLOW, 5, cv2.LINE_AA, tipLength=0.3)

    def _view_overlay(self, d, vw, label, color, stamp):
        """카메라 화면 위의 상태 테두리, 상황 표시, 범례, 사건 배너."""
        d.rectangle((0, 0, vw - 1, VIEW_H - 1), outline=color + (255,), width=6)
        chip = f"{'재생' if self.replay else '실시간'} · {LABEL_KO[label]}"
        tw = d.textlength(chip, font=self.font(15, True))
        d.rounded_rectangle((14, 14, 14 + tw + 24, 44), 10, fill=BG + (190,))
        d.ellipse((24, 24, 34, 34), fill=(RED if not self.replay else BLUE) + (255,))
        d.text((40, 29), chip, font=self.font(15, True), fill=TEXT + (255,), anchor="lm")
        legend = "노란 화살표 = 화면 이동   흰 선 = 블록 (굵은 선 = 사분면)"
        lw = d.textlength(legend, font=self.font(12))
        d.rounded_rectangle((14, VIEW_H - 40, 14 + lw + 20, VIEW_H - 16), 8, fill=BG + (170,))
        d.text((24, VIEW_H - 28), legend, font=self.font(12), fill=MUTED + (255,), anchor="lm")
        if stamp < self.banner_until:
            d.rectangle((6, 6, vw - 7, VIEW_H - 7), fill=RED + (60,))
            d.rounded_rectangle((vw // 2 - 190, VIEW_H // 2 - 50, vw // 2 + 190, VIEW_H // 2 + 50), 18,
                                fill=RED + (235,))
            d.text((vw // 2, VIEW_H // 2 - 10), f"지진 징후 감지 #{self.event_count}", font=self.font(34, True),
                   fill=TEXT + (255,), anchor="mm")
            d.text((vw // 2, VIEW_H // 2 + 26), self.events[0] if self.events else "", font=self.font(15),
                   fill=TEXT + (255,), anchor="mm")

    def _panel(self, d, x0, label, result, state, color, detail, mark, proc_ms, fps, stamp):
        a, W, pad = self.args, x0 + PANEL_W, 18
        L, R = x0 + pad, x0 + PANEL_W - pad
        T = lambda xy, s, size, fill=TEXT, bold=False, anchor="la": d.text(
            xy, s, font=self.font(size, bold), fill=fill + (255,), anchor=anchor)
        section = lambda y, s: T((L, y), s, 13, MUTED, True)
        # 머리
        T((L, 30), "지진 감지 모니터", 20, bold=True, anchor="lm")
        T((R, 30), datetime.fromtimestamp(stamp / 1e9).strftime("%H:%M:%S"), 15, MUTED, anchor="rm")
        # 상태 카드: 색 + 원형 표시 + 상태 + 이유
        d.rounded_rectangle((L, 56, R, 140), 16, fill=color + (255,))
        d.ellipse((L + 16, 74, L + 64, 122), outline=TEXT + (255,), width=3)
        T((L + 40, 98), mark, 20 if len(mark) == 1 else 15, bold=True, anchor="mm")
        T((L + 80, 92), state, 30, bold=True, anchor="ls")
        T((L + 80, 120), detail or "", 14, anchor="ls")
        # 지금 상황
        section(156, "지금 상황  ·  숫자 키 1~5")
        keys = ("still", "shake", "light", "pass", "cover")
        cw = (R - L - 4 * 6) / 5
        for i, key in enumerate(keys):
            xa = L + i * (cw + 6)
            on = key == label
            d.rounded_rectangle((xa, 176, xa + cw, 206), 9, fill=(YELLOW if on else CARD) + (255,))
            T((xa + cw / 2, 191), f"{i + 1} {LABEL_KO[key]}", 13, BG if on else TEXT, on, "mm")
        # 방향 전환: 칸 막대
        rev = result.reversals if result is not None else 0
        section(222, f"방향 전환  ·  최근 {a.window_s:g}초")
        T((R, 228), f"{rev} / {a.shakes}", 20, RED if rev >= a.shakes else TEXT, True, "rm")
        n = max(1, min(a.shakes, 12))
        sw = (R - L - (n - 1) * 5) / n
        for i in range(n):
            on = rev >= (i + 1) * a.shakes / n
            xa = L + i * (sw + 5)
            d.rounded_rectangle((xa, 246, xa + sw, 260), 5,
                                fill=((RED if rev >= a.shakes else YELLOW) if on else LINE) + (255,))
        # 화면 상태 게이지 (흰 눈금 = 기준)
        section(276, "화면 상태")
        gauges = (("무늬", result.textured_ratio if result else None, a.valid_ratio),
                  ("직전 화면과 일치", result.matched_ratio if result else None, a.valid_ratio),
                  ("원래 장면 보임", result.visible_ratio if result else None, a.visible_min))
        for i, (name, value, limit) in enumerate(gauges):
            y = 298 + i * 22
            T((L, y + 6), name, 12, MUTED, anchor="lm")
            gx0, gx1 = L + 110, R - 44
            d.rounded_rectangle((gx0, y + 1, gx1, y + 11), 5, fill=LINE + (255,))
            if value:
                d.rounded_rectangle((gx0, y + 1, gx0 + max(10, (gx1 - gx0) * value), y + 11), 5,
                                    fill=(GREEN if value >= limit else GRAY) + (255,))
            tx = gx0 + (gx1 - gx0) * limit
            d.line((tx, y - 2, tx, y + 14), fill=TEXT + (255,), width=2)
            T((R, y + 6), "-" if value is None else f"{value * 100:.0f}%", 12, anchor="rm")
        # 이동 그래프
        section(366, "화면 이동 크기  ·  최근 5초")
        T((R, 372), "● 방향 전환", 11, RED, anchor="rm")
        self._graph(d, L, 386, R - L, 52)
        # 바닥
        d.line((L, 450, R, 450), fill=LINE + (255,), width=1)
        times = "  ".join(self.events) if self.events else "없음"
        T((L, 466), f"사건 {self.event_count}  ·  {times}", 13, RED if self.event_count else MUTED, True, "lm")
        T((R, 466), f"{fps:.0f} fps · {proc_ms:.0f} ms", 12, MUTED, anchor="rm")

    def _graph(self, d, x, y, w, h):
        a = self.args
        d.rounded_rectangle((x, y, x + w, y + h), 10, fill=CARD + (255,))
        if not self.history:
            return
        t_end, span = self.history[-1][0], 5.0
        top = max(a.search * 1.5, a.shift_min * 2)
        px = lambda t: x + 6 + (w - 12) * (1 - (t_end - t) / span)
        py = lambda m: y + h - 6 - (h - 12) * min(m, top) / top
        ny = py(a.shift_min)
        for xx in range(int(x + 6), int(x + w - 6), 8):       # 점선 = shift_min(이보다 작으면 잡음)
            d.line((xx, ny, min(xx + 4, x + w - 6), ny), fill=MUTED + (160,), width=1)
        run, dots = [], []
        for t, m, rev in self.history:
            if t_end - t > span:
                continue
            if m is None:                                       # 재지 못한 구간은 끊는다
                if len(run) > 1:
                    d.line(run, fill=BLUE + (255,), width=2, joint="curve")
                run = []
                continue
            run.append((px(t), py(m)))
            if rev:
                dots.append(run[-1])
        if len(run) > 1:
            d.line(run, fill=BLUE + (255,), width=2, joint="curve")
        for cx, cy in dots:
            d.ellipse((cx - 4, cy - 4, cx + 4, cy + 4), fill=RED + (255,))


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
    meta = dict(started=datetime.now().isoformat(timespec="seconds"), git_sha=git_sha(),
                source=str(args.video) if args.video else f"camera {args.camera}",
                args={k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()}, policy=asdict(policy))
    print(f"정책: {policy}\n결과 폴더: {out}")
    dash = None
    if not args.no_window:
        make_font = load_font_maker()
        if make_font is None:
            sys.exit("화면을 그리려면 Pillow 와 한글 글꼴이 필요합니다: pip install pillow (또는 --no-window)")
        dash = Dashboard(cv2, args, make_font, replay=bool(args.video))
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
            dash.note(stamp, row["elapsed_s"], result)
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
        saved = ["probe.csv", "meta.json"] + (["view.mp4"] if view_rec and view_rec.close() else [])
        print(f"\n저장: {out}\n  " + ", ".join(saved))


if __name__ == "__main__":
    main()
