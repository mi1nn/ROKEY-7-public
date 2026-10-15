"""README 그림용 작은 그리기 도구. Pillow 만 쓴다(그래프 라이브러리 없음).

좌표는 논리 단위이고, 내부에서 SS 배로 키워 그린 뒤 줄여 저장한다(안티앨리어싱).
"""
import math

from PIL import Image, ImageDraw, ImageFont

FONT_DIR = "/usr/share/fonts/opentype/noto/"
SS = 3  # 슈퍼샘플링 배율

# 색: (테두리, 채움)
BLUE = ("#1F5FBF", "#E8F1FD")
GREEN = ("#12875A", "#E3F4EC")
ORANGE = ("#B7791F", "#FDF1DD")
PINK = ("#C2185B", "#FCE8F1")
PURPLE = ("#6D45D6", "#ECE6FB")
TEAL = ("#0E7C86", "#DFF3F5")
GRAY = ("#4A5568", "#EEF0F4")
RED = ("#C0392B", "#FCE9E7")
INK = "#1A202C"
MUTED = "#4A5568"
LINE = "#A9B2C0"
PANEL = "#F8FAFC"
PANEL_LINE = "#CBD2DC"

_fonts = {}


def font(size, bold=False, mono=False):
    key = (round(size * SS), bold, mono)
    if key not in _fonts:
        name = "NotoSansCJK-Bold.ttc" if bold else "NotoSansCJK-Regular.ttc"
        _fonts[key] = ImageFont.truetype(FONT_DIR + name, round(size * SS), index=7 if mono else 1)
    return _fonts[key]


class Canvas:
    def __init__(self, w, h, bg="white"):
        self.w, self.h = w, h
        self.img = Image.new("RGB", (w * SS, h * SS), bg)
        self.d = ImageDraw.Draw(self.img)

    def _s(self, v):
        return int(round(v * SS))

    # --- 기본 도형 -------------------------------------------------------
    def rect(self, x, y, w, h, color=GRAY, r=14, width=2, dashed=False, fill=None, outline=None):
        o = outline or color[0]
        f = color[1] if fill is None else (None if fill is False else fill)
        box = [self._s(x), self._s(y), self._s(x + w), self._s(y + h)]
        if not dashed:
            self.d.rounded_rectangle(box, radius=self._s(r), fill=f, outline=o, width=self._s(width))
            return
        if f:
            self.d.rounded_rectangle(box, radius=self._s(r), fill=f)
        self._dashed_rrect(x, y, w, h, r, o, width)

    def _dashed_rrect(self, x, y, w, h, r, c, width, dash=9, gap=7):
        s = self._s
        for (x1, y1, x2, y2) in [(x + r, y, x + w - r, y), (x + w, y + r, x + w, y + h - r),
                                  (x + w - r, y + h, x + r, y + h), (x, y + h - r, x, y + r)]:
            self.line([(x1, y1), (x2, y2)], c, width, dashed=True, dash=dash, gap=gap)
        for (cx, cy, a0) in [(x + r, y + r, 180), (x + w - r, y + r, 270), (x + w - r, y + h - r, 0), (x + r, y + h - r, 90)]:
            self.d.arc([s(cx - r), s(cy - r), s(cx + r), s(cy + r)], a0, a0 + 90, fill=c, width=s(width))

    def pill(self, x, y, w, h, color=BLUE, width=2, fill=None):
        self.rect(x, y, w, h, color, r=h / 2, width=width, fill=fill)

    def ellipse(self, cx, cy, rx, ry, color=GRAY, width=2, fill=None, outline=None):
        s = self._s
        self.d.ellipse([s(cx - rx), s(cy - ry), s(cx + rx), s(cy + ry)],
                       fill=fill if fill is not None else color[1], outline=outline or color[0], width=s(width))

    def polygon(self, pts, color=GRAY, width=2, fill=None, outline=None):
        s = self._s
        P = [(s(x), s(y)) for x, y in pts]
        self.d.polygon(P, fill=fill if fill is not None else color[1])
        self.d.line(P + [P[0]], fill=outline or color[0], width=s(width), joint="curve")

    def hatch(self, x, y, w, h, base, line, gap=9, width=1.6):
        """사선 무늬로 채운 사각형(모서리 직각)."""
        W, H = self._s(w), self._s(h)
        tmp = Image.new("RGB", (W, H), base)
        td = ImageDraw.Draw(tmp)
        g = self._s(gap)
        for k in range(-H, W, g):
            td.line([(k, H), (k + H, 0)], fill=line, width=self._s(width))
        self.img.paste(tmp, (self._s(x), self._s(y)))

    # --- 텍스트 ----------------------------------------------------------
    def text(self, x, y, s, size=14, color=INK, bold=False, mono=False, anchor="l", lh=1.4):
        """여러 줄 가능. anchor: l(왼쪽) c(가운데) r(오른쪽). y 는 첫 줄 위쪽."""
        f = font(size, bold, mono)
        for i, line in enumerate(str(s).split("\n")):
            yy = y + i * size * lh
            w = f.getlength(line) / SS
            xx = x if anchor == "l" else x - w / 2 if anchor == "c" else x - w
            self.d.text((self._s(xx), self._s(yy)), line, font=f, fill=color)

    def text_w(self, s, size=14, bold=False, mono=False):
        return font(size, bold, mono).getlength(s) / SS

    # --- 선과 화살표 -----------------------------------------------------
    def line(self, pts, color=LINE, width=2, dashed=False, dash=10, gap=7, arrow_end=False, arrow_start=False, head=9):
        s = self._s
        pts = [tuple(p) for p in pts]
        if dashed:
            for a, b in zip(pts, pts[1:]):
                self._dash_seg(a, b, color, width, dash, gap)
        else:
            self.d.line([(s(x), s(y)) for x, y in pts], fill=color, width=s(width), joint="curve")
        if arrow_end:
            self._head(pts[-2], pts[-1], color, head)
        if arrow_start:
            self._head(pts[1], pts[0], color, head)

    def _dash_seg(self, a, b, color, width, dash, gap):
        s = self._s
        (x1, y1), (x2, y2) = a, b
        length = math.hypot(x2 - x1, y2 - y1)
        if length == 0:
            return
        ux, uy = (x2 - x1) / length, (y2 - y1) / length
        t = 0.0
        while t < length:
            e = min(t + dash, length)
            self.d.line([(s(x1 + ux * t), s(y1 + uy * t)), (s(x1 + ux * e), s(y1 + uy * e))], fill=color, width=s(width))
            t += dash + gap

    def _head(self, a, b, color, size):
        s = self._s
        (x1, y1), (x2, y2) = a, b
        length = math.hypot(x2 - x1, y2 - y1) or 1
        ux, uy = (x2 - x1) / length, (y2 - y1) / length
        px, py = -uy, ux
        bx, by = x2 - ux * size, y2 - uy * size
        self.d.polygon([(s(x2), s(y2)), (s(bx + px * size * 0.5), s(by + py * size * 0.5)),
                        (s(bx - px * size * 0.5), s(by - py * size * 0.5))], fill=color)

    # --- 조합 요소 -------------------------------------------------------
    def panel(self, x, y, w, h, label, label_size=15, right=True, lx=None):
        self.rect(x, y, w, h, (PANEL_LINE, PANEL), r=22, width=2, dashed=True)
        if lx is not None:
            self.text(lx, y + 14, label, size=label_size, color=MUTED)
        elif right:
            self.text(x + w - 24, y + 14, label, size=label_size, color=MUTED, anchor="r")
        else:
            self.text(x + 24, y + 14, label, size=label_size, color=MUTED)

    def card(self, x, y, w, h, title, sub="", color=BLUE, mono_title=False, title_size=17, sub_size=13.5, pad=16):
        self.rect(x, y, w, h, color)
        self.text(x + pad, y + 12, title, size=title_size, color=color[0], bold=True, mono=mono_title)
        if sub:
            self.text(x + pad, y + 12 + title_size * 1.55, sub, size=sub_size, color=MUTED, lh=1.5)

    def badge(self, right, y, s, color=TEAL, size=12.5):
        w = self.text_w(s, size, mono=True) + 18
        self.rect(right - w, y, w, size + 12, color, r=(size + 12) / 2, width=1.6, fill="white")
        self.text(right - w + 9, y + 5, s, size=size, color=color[0], bold=True, mono=True)

    def title(self, s):
        self.text(48, 30, s, size=30, bold=True)

    def legend(self, y, items, x=48):
        for name, color, dashed in items:
            self.rect(x, y, 18, 18, color, r=5, width=1.8, dashed=dashed, fill="white" if dashed else None)
            self.text(x + 28, y - 1, name, size=14.5, color=MUTED)
            x += 28 + self.text_w(name, 14.5) + 34

    def save(self, path, out_w=2560):
        h = round(self.h * out_w / self.w)
        self.img.resize((out_w, h), Image.LANCZOS).save(path, optimize=True)
