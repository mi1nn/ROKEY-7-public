#!/usr/bin/env python3
"""YOLO 인형 학습용 웹캠 클립 촬영 도구 (ffmpeg 래퍼, macOS AVFoundation).

녹화는 ffmpeg 가 하고, 이 도구는 해상도·fps 지정, 촬영 체크리스트의 클립 ID 붙이기,
촬영 뒤 실제 해상도 확인(ffprobe), 기록(meta json)만 한다. 로봇·ROS 를 쓰지 않는다.

사용:
    python3 tools/webcam_record.py checklist                          # 찍을 장면 순서(끌개 시나리오)
    python3 tools/webcam_record.py devices                            # 카메라 번호·이름
    python3 tools/webcam_record.py modes --device USB                 # 그 카메라가 지원하는 해상도·fps
    python3 tools/webcam_record.py preview --device USB --size 1280x720 --fps 30   # 구도 맞추기 (창 닫으면 끝)
    python3 tools/webcam_record.py record --device USB --size 1280x720 --fps 30   # 시나리오 한 번에(S01)
    python3 tools/webcam_record.py split .artifacts/webcam_clips/<날짜>/S01-<시각>.mp4  # 학습·검증 자르기
    python3 tools/webcam_record.py status                             # 찍은 영상

``--device`` 는 번호(0, 1 …) 또는 이름 일부("USB", "C920")다. iPhone 이 연결되면 번호가
바뀌므로 이름을 권한다. 녹화 중 q 를 누르면 일찍 끝난다.

결과는 .artifacts/webcam_clips/<날짜>/<클립>-<시각>.mp4 와 같은 이름의 .json
(.gitignore 대상, 커밋 안 함). 지원하지 않는 해상도를 요청하거나 실제 해상도가 요청과
다르면 실패로 끝낸다(다른 해상도로 조용히 바꿔 찍지 않는다). 영상에 사람 얼굴이 찍히므로
찍히는 사람의 동의를 받고, 외부 업로드(Roboflow 등)는 팀과 정한 뒤 한다.
"""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
OUT_ROOT = ROOT / ".artifacts" / "webcam_clips"

# 촬영 체크리스트(고정 카메라 담당 계획, 2026-10-11 개정): 인형은 끌개에 올려 줄로 당겨 출구로 내보낸다.
# 영상 하나(S01)로 아래 장면을 순서대로 찍는다. (초, 거리, 장면). 마지막 val 장면들은 학습에 쓰지 않는
# 검증 구간이라 `split` 으로 따로 잘라 둔다. 초는 대략이며 장면이 끝나면 다음으로 넘어가면 된다.
SCENES = [
    ("train", 15, "배경", "빈 출구 — 인형·끌개·사람 없음"),
    ("train", 20, "배경", "사람만 지나감(빈손) 2회"),
    ("train", 20, "가까이→멀리", "빈 끌개만 줄로 당겨 출구 통과 2회 — 끌개를 인형으로 잡지 않게"),
    ("train", 30, "가까이", "끌개 위 인형, 멈춘 채 끌개 방향 돌리기(머리 앞·뒤·옆)"),
    ("train", 30, "중간", "같은 것을 중간 거리에서"),
    ("train", 30, "멀리(출구 끝)", "같은 것을 출구 끝에서 — 신뢰도가 떨어지던 거리"),
    ("train", 45, "가까이→멀리", "천천히 당겨 출구 통과 3회 — 회마다 인형 자세 바꾸기(눕힘·옆으로·엎드림)"),
    ("train", 30, "가까이→멀리", "보통 속도·빠르게 당겨 통과 각 1~2회"),
    ("train", 25, "중간", "출구 앞에서 멈췄다 다시 당김, 한 번은 뒤로 되돌아옴"),
    ("train", 25, "가까이→멀리", "당기는 사람이 끌개와 카메라 사이를 지나가며 인형을 일부 가림"),
    ("train", 25, "가까이→멀리", "조명 바꾸고(불 끄기·한쪽만) 통과 1~2회"),
    ("val", 60, "전체", "검증: 다른 사람이 당기기, 위 상황을 섞어 통과 3회(천천히·빠르게·가림)"),
]
VAL_START_S = sum(sec for split, sec, _, _ in SCENES if split == "train")
CLIPS = {"S01": ("train+val", sum(sec for _, sec, _, _ in SCENES) + 15, "전체",
                 f"끌개 시나리오 한 번에 — {VAL_START_S}초 이후는 검증 구간")}

_DEVICE = re.compile(r"\[(\d+)\] (.+)$")
_MODE = re.compile(r"(\d+)x(\d+)@\[([\d.]+) ([\d.]+)\]fps")


def parse_size(text):
    m = re.fullmatch(r"(\d+)x(\d+)", text)
    if not m:
        raise argparse.ArgumentTypeError("size must look like 1280x720")
    return int(m.group(1)), int(m.group(2))


def parse_devices(text):
    """ffmpeg -list_devices 출력에서 비디오 장치 [(번호, 이름)] 만 뽑는다."""
    devices, in_video = [], False
    for line in text.splitlines():
        if "AVFoundation video devices" in line:
            in_video = True
        elif "AVFoundation audio devices" in line:
            in_video = False
        elif in_video:
            m = _DEVICE.search(line)
            if m:
                devices.append((int(m.group(1)), m.group(2).strip()))
    return devices


def parse_modes(text):
    """ffmpeg 의 'Supported modes' 출력 -> [(너비, 높이, 최소 fps, 최대 fps)]."""
    return [(int(w), int(h), float(lo), float(hi)) for w, h, lo, hi in _MODE.findall(text)]


def resolve_device(devices, selector):
    """번호나 이름 일부로 장치 하나를 고른다. 없거나 여럿이면 실패(추측하지 않는다)."""
    if selector.isdigit():
        hits = [d for d in devices if d[0] == int(selector)]
    else:
        hits = [d for d in devices if selector.lower() in d[1].lower()]
    if len(hits) != 1:
        names = ", ".join(f"[{i}] {n}" for i, n in devices) or "없음"
        raise SystemExit(f"장치 {selector!r} 를 하나로 정할 수 없음 ({len(hits)}개 일치). 장치: {names}")
    return hits[0]


# 장치는 30 fps 를 30.000030 처럼 보고한다(2026-10-11 USB 웹캠). 요청값이 이만큼 안이면 같은 값으로 본다.
FPS_TOL = 0.01


def check_mode(modes, size, fps):
    """요청한 해상도·fps 를 장치가 지원하면 ffmpeg 에 넘길 fps 문자열을 돌려준다. 아니면 지원 목록과 함께 실패.

    범위 끝값 근처면 장치가 보고한 값 그대로를 쓴다. AVFoundation 은 반올림한 30 을 30.000030 범위로
    받아 주지 않는다.
    """
    for w, h, lo, hi in modes:
        if (w, h) != size or not lo - FPS_TOL <= fps <= hi + FPS_TOL:
            continue
        rate = lo if abs(fps - lo) <= FPS_TOL else hi if abs(fps - hi) <= FPS_TOL else fps
        return f"{rate:.6f}"
    listed = ", ".join(f"{w}x{h}@{lo:g}-{hi:g}" for w, h, lo, hi in modes) or "알 수 없음"
    raise SystemExit(f"{size[0]}x{size[1]}@{fps:g} 는 지원하지 않음. 지원: {listed}")


def check_actual(probe, size, fps, fps_tol=1.0):
    """ffprobe JSON 의 실제 해상도·fps 가 요청과 같은지. 다른 점 목록(같으면 빈 목록)."""
    streams = [s for s in probe.get("streams", []) if s.get("codec_type") == "video"]
    if not streams:
        return ["영상 스트림 없음"]
    s, problems = streams[0], []
    if (s.get("width"), s.get("height")) != size:
        problems.append(f"해상도 {s.get('width')}x{s.get('height')} != 요청 {size[0]}x{size[1]}")
    num, _, den = str(s.get("avg_frame_rate", "0/1")).partition("/")
    actual = float(num) / float(den or 1) if float(den or 1) else 0.0
    if abs(actual - fps) > fps_tol:
        problems.append(f"fps {actual:.2f} != 요청 {fps:g}")
    return problems


def clip_path(clip, now):
    return OUT_ROOT / now.strftime("%Y%m%d") / f"{clip}-{now:%H%M%S}.mp4"


def done_clips(root=OUT_ROOT):
    """확인을 통과한(meta 의 ok=True) 클립 ID -> 파일 수."""
    done = {}
    for meta in root.glob("*/*.json"):
        try:
            data = json.loads(meta.read_text())
        except (OSError, ValueError):
            continue
        if data.get("ok"):
            done[data["clip"]] = done.get(data["clip"], 0) + 1
    return done


def _ffmpeg(*args, **kw):
    return subprocess.run(["ffmpeg", "-hide_banner", *args], capture_output=True, text=True, **kw)


def list_devices():
    return parse_devices(_ffmpeg("-f", "avfoundation", "-list_devices", "true", "-i", "").stderr)


def list_modes(index):
    # 지원하지 않는 크기(1x1)로 열면 ffmpeg 가 지원 목록을 출력하고 실패한다.
    return parse_modes(_ffmpeg("-f", "avfoundation", "-video_size", "1x1", "-i", f"{index}:none",
                               "-t", "0.1", "-f", "null", "-").stderr)


def git_sha():
    try:
        return subprocess.run(["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"], capture_output=True,
                              text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def cmd_checklist(_args):
    t = 0
    print(f"영상 하나(S01)로 순서대로 찍는다. 약 {CLIPS['S01'][1]}초, {VAL_START_S}초부터 검증 구간.")
    for split, seconds, distance, what in SCENES:
        print(f"{t // 60}:{t % 60:02d}  {split:<6}{distance:<14}{what}")
        t += seconds
    done = done_clips()
    print("찍은 영상: " + (", ".join(f"{c}x{n}" for c, n in sorted(done.items())) or "없음"))


def cmd_devices(_args):
    for index, name in list_devices():
        print(f"[{index}] {name}")


def cmd_modes(args):
    index, name = resolve_device(list_devices(), args.device)
    print(f"[{index}] {name}")
    for w, h, lo, hi in list_modes(index):
        print(f"  {w}x{h}  {lo:g}~{hi:g} fps")


def cmd_preview(args):
    index, name = resolve_device(list_devices(), args.device)
    rate = check_mode(list_modes(index), args.size, args.fps)
    print(f"[{index}] {name} {args.size[0]}x{args.size[1]}@{rate} 미리보기 — 창을 닫거나 q 로 끝")
    subprocess.run(["ffplay", "-hide_banner", "-loglevel", "error", "-f", "avfoundation",
                    "-framerate", rate, "-video_size", f"{args.size[0]}x{args.size[1]}",
                    "-i", f"{index}:none", "-window_title", f"preview {name}"])


def cmd_record(args):
    split, default_seconds, distance, what = CLIPS[args.clip]
    seconds = args.seconds or default_seconds
    index, name = resolve_device(list_devices(), args.device)
    rate = check_mode(list_modes(index), args.size, args.fps)
    now = datetime.now()
    out = clip_path(args.clip, now)
    out.parent.mkdir(parents=True, exist_ok=True)
    print(f"{args.clip} ({split}) {distance}: {what}")
    print(f"[{index}] {name} {args.size[0]}x{args.size[1]}@{rate}, {seconds}초 — 일찍 끝내려면 q")
    rec = subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "warning", "-stats", "-f", "avfoundation",
                          "-framerate", rate, "-video_size", f"{args.size[0]}x{args.size[1]}",
                          "-i", f"{index}:none", "-t", str(seconds), "-c:v", "libx264", "-preset", "veryfast",
                          "-crf", "18", "-pix_fmt", "yuv420p", "-y", str(out)])
    if rec.returncode != 0 or not out.exists():
        raise SystemExit(f"녹화 실패(ffmpeg exit {rec.returncode}). macOS 카메라 권한·장치 연결을 확인")
    probe = json.loads(subprocess.run(["ffprobe", "-v", "error", "-show_streams", "-show_format", "-of", "json",
                                       str(out)], capture_output=True, text=True, check=True).stdout)
    problems = check_actual(probe, args.size, args.fps)
    meta = {"clip": args.clip, "split": split, "distance": distance, "what": what, "note": args.note,
            "ok": not problems, "problems": problems, "device": {"index": index, "name": name},
            "requested": {"width": args.size[0], "height": args.size[1], "fps": args.fps, "device_fps": rate,
                          "seconds": seconds},
            "duration_s": float(probe.get("format", {}).get("duration", 0.0)),
            "recorded_at": now.isoformat(timespec="seconds"), "file": out.name, "sha256": sha256(out),
            "git_sha": git_sha(), "ffmpeg": _ffmpeg("-version").stdout.splitlines()[0]}
    out.with_suffix(".json").write_text(json.dumps(meta, ensure_ascii=False, indent=2))
    print(f"저장: {out.relative_to(ROOT)} ({meta['duration_s']:.1f}초)")
    if problems:
        raise SystemExit("요청과 다름 — 이 클립은 찍음으로 치지 않음: " + "; ".join(problems))


def cmd_split(args):
    """찍은 영상을 학습 구간과 검증 구간 두 파일로 자른다(다시 인코딩해 자르는 시각이 정확하다)."""
    src = Path(args.file)
    if not src.exists():
        raise SystemExit(f"파일 없음: {src}")
    at = VAL_START_S if args.at is None else args.at
    for part, extra in (("train", ["-t", str(at)]), ("val", ["-ss", str(at)])):
        out = src.with_name(f"{src.stem}-{part}.mp4")
        run = subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-i", str(src), *extra,
                              "-c:v", "libx264", "-preset", "veryfast", "-crf", "18", "-pix_fmt", "yuv420p",
                              "-an", "-y", str(out)])
        if run.returncode != 0:
            raise SystemExit(f"자르기 실패(ffmpeg exit {run.returncode}): {out.name}")
        print(f"{part}: {out.relative_to(ROOT) if out.is_relative_to(ROOT) else out}")
    print(f"{at}초 기준으로 잘랐다. Roboflow 에는 train 은 학습용, val 은 검증용(Valid)으로 따로 올린다.")


def cmd_status(_args):
    done = done_clips()
    left = [c for c in CLIPS if c not in done]
    print(f"찍음 {len(CLIPS) - len(left)}/{len(CLIPS)}: " + ", ".join(f"{c}x{n}" for c, n in sorted(done.items())))
    print("남음: " + (", ".join(left) or "없음"))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("checklist", help="촬영할 클립 목록").set_defaults(fn=cmd_checklist)
    sub.add_parser("devices", help="카메라 번호·이름").set_defaults(fn=cmd_devices)
    sub.add_parser("status", help="찍은 클립 / 남은 클립").set_defaults(fn=cmd_status)
    p = sub.add_parser("split", help="찍은 영상을 학습·검증 두 파일로 자르기")
    p.add_argument("file", help="record 가 만든 .mp4")
    p.add_argument("--at", type=int, help=f"검증 시작 초(기본 {VAL_START_S})")
    p.set_defaults(fn=cmd_split)
    p = sub.add_parser("modes", help="지원 해상도·fps")
    p.add_argument("--device", required=True)
    p.set_defaults(fn=cmd_modes)
    for name, fn in (("preview", cmd_preview), ("record", cmd_record)):
        p = sub.add_parser(name)
        p.add_argument("--device", required=True, help="번호 또는 이름 일부")
        p.add_argument("--size", type=parse_size, required=True, help="예: 1280x720")
        p.add_argument("--fps", type=float, required=True)
        if name == "record":
            p.add_argument("--clip", default="S01", choices=sorted(CLIPS))
            p.add_argument("--seconds", type=int, help="기본은 체크리스트의 초")
            p.add_argument("--note", default="", help="메모(누가 들었는지, 조명 등)")
        p.set_defaults(fn=fn)
    args = ap.parse_args(argv)
    if args.cmd not in ("checklist", "status"):
        missing = [t for t in ("ffmpeg", "ffprobe", "ffplay") if shutil.which(t) is None]
        if missing:
            raise SystemExit(f"없음: {', '.join(missing)} (brew install ffmpeg)")
    args.fn(args)


if __name__ == "__main__":
    sys.exit(main())
