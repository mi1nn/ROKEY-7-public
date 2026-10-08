"""docs/schedule-timeline.csv 를 카테고리 안에서 시작 시점 순으로 정렬하고, 튜터 일정표처럼 칸을 채운 xlsx 로 만든다."""
import csv
import sys
from pathlib import Path

from openpyxl import Workbook
from openpyxl.utils import get_column_letter
from openpyxl.styles import Alignment, Border, Font, PatternFill, Side

FIXED = 7  # 카테고리 ~ 진행 상황
docs = Path(__file__).resolve().parent.parent / "docs"
csv_path, xlsx_path = docs / "schedule-timeline.csv", docs / "schedule-timeline.xlsx"

head, *rows = csv.reader(csv_path.open(encoding="utf-8"))
cats = list(dict.fromkeys(r[0] for r in rows))


def span(r):
    on = [i for i, v in enumerate(r[FIXED:]) if v]
    return on[0], on[-1]


rows.sort(key=lambda r: (cats.index(r[0]), *span(r)))
with csv_path.open("w", encoding="utf-8", newline="") as f:
    csv.writer(f, lineterminator="\n").writerows([head, *rows])

wb = Workbook()
ws = wb.active
ws.title = "Time Line"
side = Side(style="thin", color="999999")
box = Border(left=side, right=side, top=side, bottom=side)
center = Alignment(horizontal="center", vertical="center", wrap_text=True)
blue, bar = PatternFill("solid", fgColor="9FC5E8"), PatternFill("solid", fgColor="6AA84F")

days = list(dict.fromkeys(h.split()[0] for h in head[FIXED:]))
for c in range(1, FIXED + 1):
    ws.cell(1, c, head[c - 1])
    ws.merge_cells(start_row=1, start_column=c, end_row=2, end_column=c)
for d, day in enumerate(days):
    c = FIXED + 1 + d * 3
    ws.cell(1, c, day)
    ws.merge_cells(start_row=1, start_column=c, end_row=1, end_column=c + 2)
    for k, part in enumerate(("오전", "오후", "저녁")):
        ws.cell(2, c + k, part)
for row in ws["A1":ws.cell(2, FIXED + len(head[FIXED:])).coordinate]:
    for cell in row:
        cell.fill, cell.border, cell.alignment = blue, box, center
        cell.font = Font(bold=True)

for i, r in enumerate(rows, start=3):
    for c, v in enumerate(r[:FIXED], start=1):
        ws.cell(i, c, int(v) if c == FIXED else v).alignment = (
            Alignment(vertical="center", wrap_text=True) if c in (4, 6) else center
        )
    for c, v in enumerate(r[FIXED:], start=FIXED + 1):
        ws.cell(i, c).fill = bar if v else PatternFill()
    for c in range(1, FIXED + len(head[FIXED:]) + 1):
        ws.cell(i, c).border = box

start = 3  # 같은 카테고리 칸 병합
for i in range(3, len(rows) + 4):
    if i == len(rows) + 3 or rows[i - 3][0] != ws.cell(start, 1).value:
        ws.merge_cells(start_row=start, start_column=1, end_row=i - 1, end_column=1)
        ws.cell(start, 1).fill, ws.cell(start, 1).font = blue, Font(bold=True)
        start = i

for col, w in zip("ABCDEFG", (14, 14, 8, 52, 7, 22, 8)):
    ws.column_dimensions[col].width = w
for c in range(FIXED + 1, FIXED + len(head[FIXED:]) + 1):
    ws.column_dimensions[get_column_letter(c)].width = 5
ws.freeze_panes = ws.cell(3, FIXED + 1)
wb.save(xlsx_path)
print(f"{len(rows)} rows -> {xlsx_path.name}", file=sys.stderr)
