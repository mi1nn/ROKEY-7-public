#!/usr/bin/env python3
"""README 그림(docs/diagrams/*.png)을 만든다.  사용: python3 tools/diagrams/build.py [이름 ...]

필요: Pillow, 한글 글꼴 Noto Sans CJK(KR). 그림의 내용은 docs/design/architecture.md,
docs/requirements/scope.md 「지금 시나리오」, docs/process/discovery-host.md 를 따른다.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from draw import (BLUE, GREEN, GRAY, ORANGE, PINK, PURPLE, RED, TEAL, INK, LINE, MUTED,  # noqa: E402
                  PANEL, PANEL_LINE, Canvas)

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "docs", "diagrams")

LEGEND = [("인식", BLUE, False), ("지도·위치", GREEN, False), ("임무", ORANGE, False),
          ("주행·실행", PURPLE, False), ("저장·웹", PINK, False), ("하드웨어", GRAY, False), ("외부", GRAY, True)]


def lab(c, x, y, s, anchor="l", color=MUTED, size=12.5):
    c.text(x, y, s, size=size, color=color, anchor=anchor)


def vstub(c, x, y1, y2, label, both=False, flip=False, lx=8, ly=None):
    """y1 -> y2 세로 화살표. flip 이면 방향을 거꾸로, both 면 양쪽."""
    pts = [(x, y1), (x, y2)] if not flip else [(x, y2), (x, y1)]
    c.line(pts, arrow_end=True, arrow_start=both, head=8)
    if label:
        mid = ly if ly is not None else (y1 + y2) / 2 - 8
        lab(c, x + lx, mid, label)


def external(c, x, y, w, h, title, sub):
    c.rect(x, y, w, h, GRAY, dashed=True, fill="white")
    c.text(x + 16, y + 11, title, size=16.5, color=MUTED, bold=True)
    c.text(x + 16, y + 11 + 26, sub, size=13, color=MUTED)


# ---------------------------------------------------------------------------
def system_architecture():
    c = Canvas(1800, 1090)
    c.title("시스템 아키텍처")

    # 바깥
    c.rect(60, 100, 330, 72, GRAY)
    c.text(76, 111, "고정 웹캠", size=17, color=GRAY[0], bold=True)
    c.text(76, 137, "DRGO WC720 · 운영 PC 에 USB 연결", size=13, color=MUTED)
    external(c, 1320, 100, 420, 72, "운영자 브라우저", "지도 · 로봇 카메라 · 웹캠 영상 · 사건 기록")

    # 운영 PC
    c.panel(40, 200, 1720, 290, "운영 PC — Ubuntu 24.04 · ROS 2 Jazzy", lx=420)
    c.card(60, 250, 330, 100, "webcam_node", "지진 징후 · 인형 소실 판정\nYOLO26n", BLUE, True)
    c.card(470, 250, 340, 100, "mission_coordinator", "두 로봇의 임무 결정\n지도 고정 요청", ORANGE, True)
    c.card(890, 250, 340, 100, "db_writer", "현장 상태 DB (SQLite)\n운영자 명령 API", PINK, True)
    c.card(1310, 250, 420, 100, "rescue_console", "관제 웹 (Flask)\n현장 상태 DB 는 읽기 전용", PINK, True)
    c.card(470, 380, 240, 90, "report_node", "보고 저장 → 저장 확인", PINK, True)
    c.card(1310, 380, 420, 90, "mjpeg_relay · robot_view_relay", "웹캠 · 로봇 카메라 영상 중계", PINK, True)

    c.line([(225, 172), (225, 250)], arrow_end=True, head=8)
    lab(c, 233, 178, "USB")
    c.line([(1520, 172), (1520, 250)], arrow_end=True, arrow_start=True, head=8)
    lab(c, 1528, 178, "HTTP")
    c.line([(640, 350), (640, 380)], arrow_end=True, arrow_start=True, head=8)
    lab(c, 650, 355, "보고 · 저장 확인")
    c.line([(1520, 380), (1520, 350)], arrow_end=True, head=8)
    lab(c, 1528, 358, "영상")
    c.line([(1230, 292), (1310, 292)], arrow_end=True, head=8)
    c.line([(1310, 320), (1230, 320)], arrow_end=True, head=8)
    lab(c, 1270, 270, "DB 읽기", "c")
    lab(c, 1270, 326, "명령 HTTP", "c")

    # ROS 버스
    c.rect(40, 545, 1720, 42, GRAY, r=21)
    c.text(900, 556, "ROS 2 토픽 · 액션 · 서비스   —   Fast DDS Discovery Server 로 서로를 찾는다", size=15, color=GRAY[0], bold=True, anchor="c")
    vstub(c, 225, 350, 545, "지진 징후 (SiteEvent)", ly=506)
    vstub(c, 760, 350, 545, "임무 · 상태 · 고정 요청", both=True, ly=506)
    vstub(c, 1060, 350, 545, "상태 구독 · 운영자 명령", both=True, ly=506)
    vstub(c, 1520, 545, 470, "카메라 영상", ly=506)

    # TB-6 담당 PC
    c.panel(40, 640, 870, 250, "TB-6 담당 PC", lx=412)
    xs6 = [60, 275, 490, 705]
    c.card(xs6[0], 690, 195, 150, "SLAM · 지도 고정", "slam_toolbox\nmap_freeze", GREEN)
    c.card(xs6[1], 690, 195, 150, "인형 · 출구 탐지", "amr_target\ntarget_locator\nexit_locator", BLUE)
    c.card(xs6[2], 690, 195, 150, "터널 판정", "depth_image_proc\noverhead_passage", BLUE)
    c.card(xs6[3], 690, 195, 150, "주행 · 임무 실행", "Nav2 · task_input\ncommand_watchdog", PURPLE)
    vstub(c, 157, 587, 690, "/robot6/map · 고정 지도", both=True, ly=606)
    vstub(c, 372, 690, 587, "인형 · 출구 좌표", ly=606)
    vstub(c, 587, 690, 587, "구조물 목록", ly=606)
    vstub(c, 802, 587, 690, "임무 · 상태", both=True, ly=606)

    # TB-5 담당 PC
    c.panel(930, 640, 830, 250, "TB-5 담당 PC", lx=1115)
    c.card(950, 690, 250, 150, "고정 지도 위치 추정", "frozen_localization\nAMCL", GREEN)
    c.card(1220, 690, 250, 150, "터널 판정", "depth_image_proc\noverhead_passage", BLUE)
    c.card(1490, 690, 250, 150, "주행 · 임무 실행", "Nav2 · task_input\ncommand_watchdog", PURPLE)
    vstub(c, 1075, 587, 690, "고정 지도", ly=606)
    vstub(c, 1345, 690, 587, "구조물 목록", ly=606)
    vstub(c, 1615, 587, 690, "임무 · 상태", both=True, ly=606)

    # 로봇
    for x, w, name, role, ns, cols in [(60, 840, "TurtleBot4 TB-6", "탐색 담당", "/robot6", (157, 372, 587, 802)),
                                       (950, 790, "TurtleBot4 TB-5", "구조 담당", "/robot5", (1075, 1345, 1615))]:
        c.rect(x - 10 if x == 950 else x, 950, w + (10 if x == 950 else 0), 76, GRAY)
        bx = x - 10 if x == 950 else x
        c.text(bx + 18, 960, f"{name} ({role})", size=17, color=GRAY[0], bold=True)
        c.text(bx + 18, 987, "Create 3 · Raspberry Pi 4B · OAK-D-Pro · RPLIDAR A1M8", size=13, color=MUTED)
        c.badge(bx + w + (10 if x == 950 else 0) - 16, 962, ns)
    for x, label in [(157, "라이다 스캔"), (372, "RGB · depth"), (587, "RGB · depth")]:
        c.line([(x, 950), (x, 840)], arrow_end=True, head=8)
        lab(c, x + 8, 898, label)
    c.line([(802, 840), (802, 950)], arrow_end=True, head=8)
    lab(c, 810, 898, "속도 명령")
    for x, label in [(1075, "라이다 스캔"), (1345, "RGB · depth")]:
        c.line([(x, 950), (x, 840)], arrow_end=True, head=8)
        lab(c, x + 8, 898, label)
    c.line([(1615, 840), (1615, 950)], arrow_end=True, head=8)
    lab(c, 1623, 898, "속도 명령")

    c.legend(1050, LEGEND)
    c.save(os.path.join(OUT, "system_architecture.png"))


# ---------------------------------------------------------------------------
def network():
    c = Canvas(1500, 800)
    c.title("네트워크 구성")
    c.panel(40, 100, 1420, 640, "같은 무선망 — Fast DDS Discovery Server 로 노드를 찾는다", right=False)

    c.rect(560, 160, 380, 120, ORANGE)
    c.text(580, 172, "Discovery 전용 PC", size=18, color=ORANGE[0], bold=True)
    c.text(580, 202, "Fast DDS Discovery Server\n서버 ID 0 · 2026-10-15 부터 사용", size=13.5, color=MUTED, lh=1.5)
    c.badge(920, 174, "UDP :11811", ORANGE)

    cols = [(100, 320, "운영 PC", "운영 노드 · 임무 조정기\n관제 웹 · 현장 상태 DB", PINK),
            (590, 320, "TB-6 담당 PC", "SLAM · Nav2 · 인형 탐지\n지도 고정", PURPLE),
            (1080, 320, "TB-5 담당 PC", "AMCL · Nav2\n통로·터널 판정", PURPLE)]
    for x, w, t, s, col in cols:
        c.card(x, 400, w, 110, t, s, col)
    c.line([(750, 280), (750, 340)], arrow_start=True, arrow_end=True, head=8)
    c.line([(260, 340), (1240, 340)])
    for x, w, *_ in cols:
        cx = x + w / 2
        if cx != 750:
            c.line([(cx, 340), (cx, 400)], arrow_end=True, arrow_start=False, head=8)
        else:
            c.line([(750, 340), (750, 400)], arrow_end=True, head=8)
    lab(c, 760, 300, "ROS_DISCOVERY_SERVER")

    c.rect(100, 610, 320, 90, GRAY)
    c.text(118, 622, "고정 웹캠", size=17, color=GRAY[0], bold=True)
    c.text(118, 648, "DRGO WC720 (USB)", size=13.5, color=MUTED)
    c.line([(260, 610), (260, 510)], arrow_end=True, head=8)
    lab(c, 268, 556, "USB")

    for x, w, t, ns, sid in [(590, 320, "TurtleBot4 TB-6", "/robot6", "ID 6"),
                             (1080, 320, "TurtleBot4 TB-5", "/robot5", "ID 5")]:
        c.rect(x, 610, w, 90, GRAY)
        c.text(x + 18, 622, t, size=17, color=GRAY[0], bold=True)
        c.text(x + 18, 648, f"Raspberry Pi 4B · 온보드 서버 {sid}", size=13.5, color=MUTED)
        c.badge(x + w - 14, 624, ns)
        c.line([(x + w / 2, 510), (x + w / 2, 610)], arrow_end=True, arrow_start=True, head=8)
        lab(c, x + w / 2 + 8, 556, "ROS 토픽 (DDS)")
    c.save(os.path.join(OUT, "network.png"))



# ---------------------------------------------------------------------------
def mission_flow():
    c = Canvas(1700, 940)
    c.title("동작 순서도")
    rows = [
        (BLUE, "웹캠 판정", "webcam_node · 운영 PC",
         ["영상 입력", "화면 왕복 횟수 세기", "지진 징후 (DISASTER_SIGN)", "탐색 자동 시작"],
         "판단 보류: 손 가림 · 어두움 · 렌즈 막힘 · 영상 끊김은 지진 근거로 쓰지 않는다"),
        (GREEN, "TB-6 탐색", "SLAM · frontier 이동",
         ["frontier 선택", "이동", "지도 갱신", "출구 QR 기록", "인형 확정"],
         "출구를 찾아도 탐색은 계속한다. 더 갈 frontier 가 없으면 탐색 완료"),
        (ORANGE, "보고 · 1차 고정", "조정기 · report_node · map_freeze",
         ["보고 저장 (PERSISTED)", "1차 지도 고정 (R0+1)", "TB-5 자동 출동"],
         "저장 확인 전에는 TB-5 가 출발하지 않는다"),
        (PURPLE, "TB-5 구조", "task_input · Nav2",
         ["인형 앞 접근 지점 대기", "2차 지도 고정 (R0+2)", "위치 다시 맞춤", "가까운 출구로 이동", "출구 도착"],
         "탐색이 끝났는데 인형이 없으면 출발하지 않고 NO_SURVIVOR_FOUND · QR 출구가 없으면 TB-5 출발 자리가 출구"),
        (GREEN, "마무리", "TB-6 · 조정기",
         ["도킹 접근 지점으로 이동", "MISSION_FINISHED"],
         "구출 확인은 TB-5 의 출구 도착이다. 도킹은 사람이 한다"),
    ]
    y = 100
    for color, title, sub, steps, note in rows:
        c.rect(48, y, 310, 92, color)
        c.text(66, y + 14, title, size=19, color=color[0], bold=True)
        c.text(66, y + 48, sub, size=13.5, color=MUTED)
        x = 400
        for i, st in enumerate(steps):
            w = c.text_w(st, 15.5) + 40
            c.pill(x, y + 8, w, 46, color, fill="white")
            c.text(x + w / 2, y + 20, st, size=15.5, color=color[0], bold=True, anchor="c")
            x += w
            if i < len(steps) - 1:
                c.line([(x + 4, y + 31), (x + 30, y + 31)], arrow_end=True, head=8)
                x += 34
        c.text(400, y + 66, note, size=14, color=MUTED)
        y += 134
    # 아래 카드 두 장
    for x, w, t, body in [(48, 790, "통로 · 터널 판정",
                           "머리 위 여유가 46 cm 이상이면 통과하고, 미만이거나 판정 불가면 멈추고 보고한다.\n두 로봇 모두 깊이 카메라로 잰다. 라이다가 못 보는 높이 띠를 깊이로만 막는다."),
                          (860, 792, "정지 규칙",
                           "위치 · 지도 · 시각을 모르면 새 명령을 막고 이유를 남긴다.\n취소한 뒤 실제 정지를 관측하고, 못 하면 STOP_UNCONFIRMED 로 남긴다.")]:
        c.rect(x, 790, w, 120, ("#D5DBE5", "#F8FAFC"), r=18)
        c.text(x + 24, 806, t, size=17, bold=True)
        c.text(x + 24, 838, body, size=14, color=MUTED, lh=1.6)
    c.save(os.path.join(OUT, "mission_flow.png"))


# ---------------------------------------------------------------------------
def command_flow():
    names = [("웹캠 판정", BLUE), ("관제 웹", PINK), ("db_writer", PINK), ("임무 조정기", ORANGE),
             ("실행기 task_input", PURPLE), ("Nav2", PURPLE), ("command_watchdog", PURPLE), ("TurtleBot4", GRAY)]
    W, H = 1700, 1240
    c = Canvas(W, H)
    c.title("명령 처리 시퀀스")
    xs = [150 + i * 200 for i in range(8)]
    top = 210 + 11 * 64 + 6
    # 정지 관문 패널(맨 아래, 생명선보다 먼저 그린다)
    c.rect(xs[3] - 110, top, xs[7] - xs[3] + 220, 285, RED, r=20, dashed=True, fill="#FFF8F7")
    c.text(xs[3] + 16, top + 12, "정지 관문 — 임무가 끝나거나 취소되거나 시계가 만료될 때 항상", size=15, color=RED[0], bold=True)
    for x, (n, col) in zip(xs, names):
        c.rect(x - 88, 100, 176, 56, col)
        c.text(x, 118, n, size=14.5, color=col[0], bold=True, anchor="c")
        c.line([(x, 156), (x, H - 40)], color="#CBD2DC", width=2, dashed=True)

    def msg(y, a, b, label, color="#6B7585", text_color=INK, dashed=False, sub=None):
        x1, x2 = xs[a], xs[b]
        d = 1 if x2 > x1 else -1
        c.line([(x1 + 3 * d, y), (x2 - 3 * d, y)], color=color, width=2.2, dashed=dashed, arrow_end=True, head=10)
        c.text((x1 + x2) / 2, y - 24, label, size=14, color=text_color, anchor="c")
        if sub:
            c.text((x1 + x2) / 2, y + 8, sub, size=13, color=text_color, anchor="c")

    y = 210
    steps = [
        (0, 3, "① SiteEvent · 지진 징후 (DISASTER_SIGN)", False),
        (1, 2, "② 운영자 명령: POST /api/operator-commands", False),
        (2, 3, "③ SubmitOperatorCommand", False),
        (3, 4, "④ ExecuteTask + lease (명령 권한)", False),
        (4, 5, "⑤ NavigateToPose 목표", False),
        (5, 6, "⑥ cmd_vel", False),
        (6, 7, "⑦ 속도 명령 (만료 · stop fence 검사)", False),
        (7, 4, "⑧ status · pose · 도착", True),
        (4, 3, "⑨ 상태 · 결과 (Result)", True),
        (3, 2, "⑩ SystemState · 상태 기록", True),
        (2, 1, "⑪ DB 읽기 → 관제 화면", True),
    ]
    for a, b, label, dashed in steps:
        msg(y, a, b, label, "#6B7585" if not dashed else "#8792A2", INK if not dashed else MUTED, dashed)
        y += 64
    yy = top + 90
    msg(yy, 3, 4, "⑫ 취소 · 만료", RED[0], RED[0])
    yy += 62
    c.rect(xs[4] - 100, yy - 18, 200, 40, RED, r=10, fill="white")
    c.text(xs[4], yy - 8, "실제 정지를 관측", size=14, color=RED[0], bold=True, anchor="c")
    yy += 70
    msg(yy, 4, 3, "⑬ stop_confirmed", RED[0], RED[0], True, sub="못 하면 STOP_UNCONFIRMED → 새 임무 차단")
    c.save(os.path.join(OUT, "command_flow.png"))


# ---------------------------------------------------------------------------
def map_flow():
    c = Canvas(1700, 760)
    c.title("지도 고정 흐름")
    cols = [270, 730, 1190]
    cw = 420
    heads = ["탐색 중", "1차 고정 (R0+1)", "2차 고정 (R0+2)"]
    for x, h in zip(cols, heads):
        c.pill(x + 60, 100, cw - 120, 42, GRAY, fill="white")
        c.text(x + cw / 2, 110, h, size=16.5, color=GRAY[0], bold=True, anchor="c")
    lanes = [
        (GREEN, "TB-6", "탐색 담당", [
            ("SLAM 으로 지도 작성", "frontier 를 따라 이동\n출구 QR 기록"),
            ("정지 상태에서 고정", "고정 뒤에도 SLAM 은 계속한다"),
            ("고정 자세 수신", "사람이 SLAM 을 끄고\nAMCL 로 바꾼다")]),
        (ORANGE, "조정기 · map_freeze", "지도 고정 담당", [
            ("탐색 경계 계산", "/robot6/map 을 읽는다"),
            ("고정 지도 판 1 발행", "조건: 인형 확정 + 보고 저장(PERSISTED)\n지도만 복사한다"),
            ("고정 지도 판 2 발행", "조건: 탐색 완료 (더 갈 경계 없음)\n세 번째 요청은 거절")]),
        (PURPLE, "TB-5", "구조 담당", [
            ("대기", "지도 없음 (MapState UNKNOWN)\n출발 자리에서 기다린다"),
            ("위치 추정 후 자동 출동", "출발 자리 기준으로 AMCL\n인형 앞 접근 지점으로"),
            ("멈춘 채 위치 다시 맞춤", "완성 지도 위에서\n가까운 출구로 사람과 이동")]),
    ]
    ys = [170, 340, 510]
    for (col, name, sub, cells), y in zip(lanes, ys):
        c.rect(48, y, 190, 130, col)
        c.text(66, y + 18, name, size=16, color=col[0], bold=True)
        c.text(66, y + 50, sub, size=13.5, color=MUTED)
        for i, (t, b) in enumerate(cells):
            c.rect(cols[i], y, cw, 130, col)
            c.text(cols[i] + 18, y + 16, t, size=17, color=col[0], bold=True)
            c.text(cols[i] + 18, y + 50, b, size=14, color=MUTED, lh=1.55)
            if i < 2:
                c.line([(cols[i] + cw + 4, y + 65), (cols[i + 1] - 4, y + 65)], arrow_end=True, head=9)
    # 세로 연결
    for x in (cols[1] + cw / 2, cols[2] + cw / 2):
        c.line([(x, ys[1] + 134), (x, ys[2] - 4)], arrow_end=True, head=9)
    c.line([(cols[2] + cw / 2, ys[1] - 4), (cols[2] + cw / 2, ys[0] + 134)], arrow_end=True, head=9)
    c.rect(48, 670, 1604, 52, RED, r=14, fill="#FFF8F7")
    c.text(72, 684, "고정은 한 회차에 두 번까지다. 기준 지도는 따로 없고 TB-6 의 SLAM 지도가 두 로봇의 공통 지도다.", size=15, color=RED[0], bold=True)
    c.save(os.path.join(OUT, "map_flow.png"))


# ---------------------------------------------------------------------------
def hardware_stack():
    c = Canvas(1700, 790)
    c.title("장비 구성")
    PH = 600
    c.panel(48, 100, 640, PH, "TurtleBot4 × 2  (TB-6 탐색 · TB-5 구조, 같은 구성)")
    items = [("iRobot Create 3", "구동 베이스 · 도킹 · 속도 명령 수신", GRAY),
             ("Raspberry Pi 4B", "ROS 2 Jazzy · 온보드 Discovery 서버", GRAY),
             ("UI 보드", "LED · 버튼 · 128×64 디스플레이", GRAY),
             ("RPLIDAR A1M8", "360° 2D 라이다 — SLAM · 장애물 회피", GREEN),
             ("OAK-D-Pro", "RGB + depth 카메라 — 인형 탐지 · 통로 높이", BLUE)]
    y = 150
    for t, s_, col in items:
        c.card(76, y, 584, 90, t, s_, col, title_size=18, sub_size=14.5)
        y += 106
    c.panel(780, 100, 500, PH, "PC × 4  (같은 무선망)")
    pcs = [("운영 PC", "임무 조정기 · 현장 상태 DB · 관제 웹\n웹캠 판정 · 보고 저장", PINK),
           ("TB-6 담당 PC", "SLAM · Nav2 · 인형 탐지 · 지도 고정", PURPLE),
           ("TB-5 담당 PC", "고정 지도 위치 추정(AMCL) · Nav2\n통로 · 터널 판정", PURPLE),
           ("Discovery 전용 PC", "Fast DDS Discovery Server (ID 0)", ORANGE)]
    y = 150
    for t, s_, col in pcs:
        c.card(808, y, 444, 116, t, s_, col, title_size=18, sub_size=14.5)
        y += 132
    c.panel(1370, 100, 282, PH, "현장 고정 장비")
    site = [("웹캠", "DRGO WC720\n지진 징후 · 인형 소실 감시"),
            ("출구 QR", "허용 목록 글자와\n정확히 같은 것만 출구"),
            ("구조 대상", "인형")]
    y = 150
    for t, s_ in site:
        c.card(1388, y, 246, 130, t, s_, GRAY, title_size=18, sub_size=14.5)
        y += 150
    c.line([(688, 420), (780, 420)], arrow_end=True, arrow_start=True, head=9)
    c.text(734, 386, "무선", size=13, color=MUTED, anchor="c")
    c.text(734, 442, "ROS 토픽", size=13, color=MUTED, anchor="c")
    c.line([(1370, 208), (1280, 208)], arrow_end=True, head=9)
    c.text(1325, 176, "USB", size=13, color=MUTED, anchor="c")
    c.save(os.path.join(OUT, "hardware_stack.png"))


# ---------------------------------------------------------------------------
def arena_map():
    """작업대 배치도. 치수는 docs/diagrams/arena_map_source.svg (1 단위 = 1 cm)를 그대로 옮긴 것."""
    BR = "#8A6A3F"          # 벽·박스 테두리
    TAN, TAN_L = "#E7DCCB", "#C9A978"
    c = Canvas(1500, 1370)
    c.title("작업대 배치")
    c.text(48, 76, "전체 343 × 557 cm · 격자 한 칸 50 cm", size=14.5, color=MUTED)
    S = 2.0
    OX, OY = 150, 190

    def X(v):
        return OX + v * S

    def Y(v):
        return OY + v * S

    def R(x, y, w, h):
        return X(x), Y(y), w * S, h * S

    # 바닥: 메인 구역(흰색) / 로봇 구역(연한 파랑)
    c.rect(*R(0, 0, 343, 356), GRAY, r=0, width=0, fill="#FFFFFF", outline="#FFFFFF")
    c.rect(*R(0, 356, 343, 201), GRAY, r=0, width=0, fill="#EAF1F8", outline="#EAF1F8")
    for gx in range(0, 344, 50):          # 50 cm 격자
        c.line([(X(gx), Y(0)), (X(gx), Y(557))], color="#DDE3EA", width=1)
    for gy in range(0, 558, 50):
        c.line([(X(0), Y(gy)), (X(343), Y(gy))], color="#DDE3EA", width=1)

    # 터널 2 (통과 불가)
    c.hatch(*R(140, 230, 72, 81), "#F7E1DD", "#D86A5D", gap=10)
    c.line([(X(168), Y(230)), (X(168), Y(311))], color="#C23A2B", width=4, dashed=True, dash=12, gap=6)
    c.rect(*R(140, 230, 72, 81), RED, r=0, width=1.6, fill=False)
    # 터널 1 (통과 가능)
    c.hatch(*R(275, 311, 68, 45), "#DDF1E8", "#7FC4A3", gap=10)
    c.line([(X(262), Y(334)), (X(343), Y(334))], color="#1F8A5B", width=4, dashed=True, dash=12, gap=6)
    # 막힌 칸(흰 박스 칸)
    c.hatch(*R(140, 311, 135, 45), TAN, TAN_L, gap=9)
    c.rect(*R(140, 311, 135, 45), (BR, TAN), r=0, width=5, fill=False)
    c.rect(*R(182, 316, 46, 36), GRAY, r=3, width=1.6, fill="#FFFFFF", outline="#5D6B78")
    # 박스 기둥
    c.hatch(*R(155, 167, 50, 63), TAN, TAN_L, gap=9)
    c.rect(*R(155, 167, 50, 63), (BR, TAN), r=0, width=5, fill=False)
    # 관제 박스
    c.hatch(*R(0, 0, 80, 140), TAN, TAN_L, gap=9)
    c.rect(*R(0, 0, 80, 140), (BR, TAN), r=0, width=5, fill=False)
    # 북쪽 벽 토막 + 기둥
    c.line([(X(236), Y(0)), (X(236), Y(92))], color=BR, width=6)
    c.hatch(*R(224, 92, 26, 28), TAN, TAN_L, gap=9)
    c.rect(*R(224, 92, 26, 28), (BR, TAN), r=0, width=5, fill=False)
    # 바깥 벽 + 구역 나누는 벽
    c.rect(*R(0, 0, 343, 557), (BR, "#FFFFFF"), r=0, width=8, fill=False)
    c.line([(X(0), Y(356)), (X(140), Y(356))], color=BR, width=8)

    # 구역 이름
    c.text(X(330), Y(177), "메인 구역", size=15, color=MUTED, anchor="r")
    c.text(X(171), Y(432), "로봇 구역", size=15, color=MUTED, anchor="c")

    # 로봇과 도크
    for cx, name in ((88, "TB-6"), (270, "TB-5")):
        c.rect(X(cx - 12), Y(545), 24 * S, 8 * S, GRAY, r=4, width=0, fill="#B5BCC7", outline="#B5BCC7")
        c.ellipse(X(cx), Y(520), 17 * S, 17 * S, GRAY, width=3, fill="#FFFFFF", outline="#22303C")
        c.text(X(cx), Y(520) - 10, name, size=16, color="#22303C", bold=True, anchor="c")
    # 인형
    c.ellipse(X(120), Y(270), 7 * S, 7 * S, ORANGE, width=2, fill="#D98A1C", outline="#9C6212")

    # 번호표
    def num(n, x, y, color=INK):
        c.ellipse(x, y, 14, 14, GRAY, width=0, fill=color, outline=color)
        c.text(x, y - 10, str(n), size=15, color="#FFFFFF", bold=True, anchor="c")

    c.rect(X(5), Y(44), 70 * S, 58 * S, GRAY, r=8, width=0, fill="#FFFFFF", outline="#FFFFFF")
    num(1, X(40), Y(30))
    c.text(X(40), Y(58), "관제 박스", size=15, bold=True, anchor="c")
    c.text(X(40), Y(78), "노트북 · 공유기\n카메라 삼각대", size=12.5, color=MUTED, anchor="c", lh=1.35)
    num(2, X(180), Y(198))
    c.text(X(180), Y(150), "박스 기둥", size=14, bold=True, anchor="c")
    num(3, X(237), Y(106))
    c.text(X(256), Y(112) - 8, "기둥", size=14, bold=True)
    num(4, X(176), Y(270), RED[0])
    c.text(X(220), Y(255), "터널 2", size=16, color=RED[0], bold=True)
    c.text(X(220), Y(272), "통과 불가", size=13, color=RED[0])
    num(5, X(309), Y(345), GREEN[0])
    c.text(X(309), Y(279), "터널 1", size=16, color=GREEN[0], bold=True, anchor="c")
    c.text(X(309), Y(296), "높이 47 cm", size=13, color=GREEN[0], anchor="c")
    num(6, X(160), Y(333))
    c.text(X(205), Y(344) - 4, "흰 박스", size=12.5, color=MUTED, anchor="c")
    num(7, X(120), Y(255), ORANGE[0])
    c.text(X(108), Y(262), "인형", size=14, color=ORANGE[0], bold=True, anchor="r")
    num(8, X(88), Y(480))
    num(8, X(270), Y(480))

    # 치수선
    d = "#5D6B78"
    c.line([(X(0), OY - 36), (X(343), OY - 36)], color=d, width=1.6)
    c.line([(X(0), OY - 44), (X(0), OY - 28)], color=d, width=1.6)
    c.line([(X(343), OY - 44), (X(343), OY - 28)], color=d, width=1.6)
    c.text(X(171.5), OY - 66, "343 cm", size=15, color=d, anchor="c", mono=True)
    xl = X(343) + 38
    c.line([(xl, Y(0)), (xl, Y(557))], color=d, width=1.6)
    for yy in (0, 356, 557):
        c.line([(xl - 8, Y(yy)), (xl + 8, Y(yy))], color=d, width=1.6)
    c.text(xl + 14, Y(178) - 10, "356 cm", size=15, color=d, mono=True)
    c.text(xl + 14, Y(456) - 10, "201 cm", size=15, color=d, mono=True)

    # 오른쪽 목록
    lx, lw = 1010, 440
    c.text(lx, 150, "구성", size=20, bold=True)
    items = [
        (1, INK, "관제 박스", "80 × 140 cm · 노트북, 공유기, 카메라 삼각대"),
        (2, INK, "박스 기둥", "50 × 63 cm"),
        (3, INK, "기둥과 벽", "기둥 26 × 28 cm, 위쪽 벽 92 cm"),
        (4, RED[0], "터널 2", "72 × 81 cm · 통과 불가"),
        (5, GREEN[0], "터널 1", "68 × 45 cm · 높이 47 cm, 통과 가능"),
        (6, INK, "흰 박스 칸", "135 × 45 cm 막힌 칸, 안에 흰 박스 46 × 36 cm"),
        (7, ORANGE[0], "인형", "메인 구역, 터널 2 바로 왼쪽"),
        (8, INK, "TB-6 · TB-5", "로봇 구역 아래쪽, 도크에서 출발"),
    ]
    y = 190
    for n, col, t, sub in items:
        c.rect(lx, y, lw, 84, (PANEL_LINE, "#FFFFFF"), r=14, width=1.6)
        c.ellipse(lx + 34, y + 42, 14, 14, GRAY, width=0, fill=col, outline=col)
        c.text(lx + 34, y + 32, str(n), size=15, color="#FFFFFF", bold=True, anchor="c")
        c.text(lx + 64, y + 14, t, size=16.5, bold=True, color=col)
        c.text(lx + 64, y + 44, sub, size=13.5, color=MUTED)
        y += 98
    # 범례
    ly = y + 20
    c.text(lx, ly, "표시", size=20, bold=True)
    ly += 44
    c.hatch(lx, ly, 40, 26, TAN, TAN_L, gap=8)
    c.rect(lx, ly, 40, 26, (BR, TAN), r=0, width=3, fill=False)
    c.text(lx + 54, ly + 2, "박스 · 막힌 곳", size=14, color=MUTED)
    c.hatch(lx + 210, ly, 40, 26, "#DDF1E8", "#7FC4A3", gap=8)
    c.text(lx + 264, ly + 2, "통과 가능", size=14, color=MUTED)
    ly += 44
    c.hatch(lx, ly, 40, 26, "#F7E1DD", "#D86A5D", gap=8)
    c.text(lx + 54, ly + 2, "통과 불가", size=14, color=MUTED)
    c.rect(lx + 210, ly, 40, 26, GRAY, r=0, width=0, fill="#EAF1F8", outline="#EAF1F8")
    c.text(lx + 264, ly + 2, "로봇 구역", size=14, color=MUTED)
    c.text(lx, ly + 52, "격자 한 칸 50 cm", size=13, color=MUTED)
    c.save(os.path.join(OUT, "arena_map.png"))


BUILDERS = {"system_architecture": system_architecture, "network": network, "mission_flow": mission_flow,
            "command_flow": command_flow, "map_flow": map_flow, "hardware_stack": hardware_stack,
            "arena_map": arena_map}

if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    names = sys.argv[1:] or list(BUILDERS)
    for n in names:
        BUILDERS[n]()
        print("ok", n)
