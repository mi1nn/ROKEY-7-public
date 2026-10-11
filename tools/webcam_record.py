#!/usr/bin/env python3
"""YOLO 인형 학습용 웹캠 클립 촬영 도구 (ffmpeg 래퍼, macOS AVFoundation).

녹화는 ffmpeg 가 하고, 이 도구는 해상도·fps 지정, 촬영 체크리스트의 클립 ID 붙이기,
촬영 뒤 실제 해상도 확인(ffprobe), 기록(meta json)만 한다. 로봇·ROS 를 쓰지 않는다.

사용:
    python3 tools/webcam_record.py checklist                          # 촬영할 클립 목록
    python3 tools/webcam_record.py devices                            # 카메라 번호·이름
    python3 tools/webcam_record.py modes --device USB                 # 그 카메라가 지원하는 해상도·fps
    python3 tools/webcam_record.py preview --device USB --size 1280x720 --fps 30   # 구도 맞추기 (창 닫으면 끝)
    python3 tools/webcam_record.py record --clip C06 --device USB --size 1280x720 --fps 30
    python3 tools/webcam_record.py status                             # 찍은 클립 / 남은 클립

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

# 촬영 체크리스트(고정 카메라 담당 계획, 2026-10-11). split: train 은 학습, val 은 학습에 쓰지 않는 검증용.
# seconds 는 기본 촬영 길이. 초당 2~3장을 뽑으면 30초 클립 하나가 60~90장이다.
CLIPS = {
    "C01": ("train", 20, "배경", "빈 출구 — 인형·사람 없음"),
    "C02": ("train", 30, "배경", "사람만 지나감(인형 없음) — 빈손, 가방 등"),
    "C03": ("train", 30, "가까이", "인형을 바닥에 두고 자세 바꾸기(눕힘·세움·엎드림·옆으로)"),
    "C04": ("train", 30, "중간", "인형 바닥, 자세 바꾸기"),
    "C05": ("train", 30, "멀리(출구 끝)", "인형 바닥, 자세 바꾸기 — 신뢰도가 떨어지던 거리"),
    "C06": ("train", 30, "가까이→멀리", "인형을 안고 천천히 출구로 지나감(여러 번 왕복)"),
    "C07": ("train", 30, "가까이→멀리", "인형을 한 손으로 들고 보통 속도로 지나감"),
    "C08": ("train", 30, "가까이→멀리", "인형이 몸에 일부 가려지게 안고 지나감(옆구리·등 쪽)"),
    "C09": ("train", 30, "가까이→멀리", "빠르게 지나감 — 번진 프레임 포함"),
    "C10": ("train", 30, "중간", "출구 앞에서 머뭇거리다 되돌아옴(인형 든 채)"),
    "C11": ("train", 30, "전체", "조명 바꾸기(불 끄고 켜기·한쪽만) — 인형 바닥 + 들고 이동"),
    "C12": ("train", 30, "전체", "두 사람 — 한 명은 인형, 한 명은 빈손으로 같이 지나감"),
    "V01": ("val", 30, "전체", "검증: 안고 지나감 + 바닥 멀리(섞어서)"),
    "V02": ("val", 30, "전체", "검증: 한 손 들기 + 가려짐 + 사람만 지나감"),
    "V03": ("val", 30, "전체", "검증: 학습 클립과 다른 사람·다른 조명"),
}

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


def check_mode(modes, size, fps):
    """요청한 해상도·fps 를 장치가 지원하는지. 아니면 지원 목록과 함께 실패."""
    if not any((w, h) == size and lo <= fps <= hi for w, h, lo, hi in modes):
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
    done = done_clips()
    print(f"{'클립':<5}{'용도':<7}{'초':>4}  {'거리':<12}내용")
    for clip, (split, seconds, distance, what) in CLIPS.items():
        mark = f"  [찍음 {done[clip]}]" if clip in done else ""
        print(f"{clip:<5}{split:<7}{seconds:>4}  {distance:<12}{what}{mark}")


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
    check_mode(list_modes(index), args.size, args.fps)
    print(f"[{index}] {name} {args.size[0]}x{args.size[1]}@{args.fps:g} 미리보기 — 창을 닫거나 q 로 끝")
    subprocess.run(["ffplay", "-hide_banner", "-loglevel", "error", "-f", "avfoundation",
                    "-framerate", f"{args.fps:g}", "-video_size", f"{args.size[0]}x{args.size[1]}",
                    "-i", f"{index}:none", "-window_title", f"preview {name}"])


def cmd_record(args):
    split, default_seconds, distance, what = CLIPS[args.clip]
    seconds = args.seconds or default_seconds
    index, name = resolve_device(list_devices(), args.device)
    check_mode(list_modes(index), args.size, args.fps)
    now = datetime.now()
    out = clip_path(args.clip, now)
    out.parent.mkdir(parents=True, exist_ok=True)
    print(f"{args.clip} ({split}) {distance}: {what}")
    print(f"[{index}] {name} {args.size[0]}x{args.size[1]}@{args.fps:g}, {seconds}초 — 일찍 끝내려면 q")
    rec = subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "warning", "-stats", "-f", "avfoundation",
                          "-framerate", f"{args.fps:g}", "-video_size", f"{args.size[0]}x{args.size[1]}",
                          "-i", f"{index}:none", "-t", str(seconds), "-c:v", "libx264", "-preset", "veryfast",
                          "-crf", "18", "-pix_fmt", "yuv420p", "-y", str(out)])
    if rec.returncode != 0 or not out.exists():
        raise SystemExit(f"녹화 실패(ffmpeg exit {rec.returncode}). macOS 카메라 권한·장치 연결을 확인")
    probe = json.loads(subprocess.run(["ffprobe", "-v", "error", "-show_streams", "-show_format", "-of", "json",
                                       str(out)], capture_output=True, text=True, check=True).stdout)
    problems = check_actual(probe, args.size, args.fps)
    meta = {"clip": args.clip, "split": split, "distance": distance, "what": what, "note": args.note,
            "ok": not problems, "problems": problems, "device": {"index": index, "name": name},
            "requested": {"width": args.size[0], "height": args.size[1], "fps": args.fps, "seconds": seconds},
            "duration_s": float(probe.get("format", {}).get("duration", 0.0)),
            "recorded_at": now.isoformat(timespec="seconds"), "file": out.name, "sha256": sha256(out),
            "git_sha": git_sha(), "ffmpeg": _ffmpeg("-version").stdout.splitlines()[0]}
    out.with_suffix(".json").write_text(json.dumps(meta, ensure_ascii=False, indent=2))
    print(f"저장: {out.relative_to(ROOT)} ({meta['duration_s']:.1f}초)")
    if problems:
        raise SystemExit("요청과 다름 — 이 클립은 찍음으로 치지 않음: " + "; ".join(problems))


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
    p = sub.add_parser("modes", help="지원 해상도·fps")
    p.add_argument("--device", required=True)
    p.set_defaults(fn=cmd_modes)
    for name, fn in (("preview", cmd_preview), ("record", cmd_record)):
        p = sub.add_parser(name)
        p.add_argument("--device", required=True, help="번호 또는 이름 일부")
        p.add_argument("--size", type=parse_size, required=True, help="예: 1280x720")
        p.add_argument("--fps", type=float, required=True)
        if name == "record":
            p.add_argument("--clip", required=True, choices=sorted(CLIPS))
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
