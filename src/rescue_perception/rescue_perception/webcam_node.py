"""고정 웹캠 ROS 노드(webcam_node). 웹캠 1대를 읽어 지진 판정과 출구 구조 판정을 돌리고 결과를 발행한다.

이 파일은 '입출력'만 맡는다. 판정 규칙은 다른 파일에 있다.
    지진 판정        → disaster_sign.py
    출구 구조 판정   → target_watch.py
    감시 요청 처리 등 → webcam_service.py

들어오는 것
    - 웹캠 프레임 (0.2초마다 1장, 640×480)
    - 파라미터 YAML (모든 값이 필수, 코드에 기본값 없음)
    - 모델 파일 webcam-yolo26n-best.pt (해시가 YAML 값과 다르면 기동하지 않음)
    - SetCameraWatch 서비스 요청 (감시 걸기·해제, 조정기가 보냄)
나가는 것
    - CameraStatus     : 카메라 상태 (ONLINE / OFFLINE / DEGRADED / UNKNOWN)
    - SiteEvent        : 사건 (DISASTER_SIGN 지진, TARGET_DISAPPEARED_CANDIDATE 구조 후보)
    - TargetPresence   : 감시 중일 때 프레임마다 인형 상태
    - CompressedImage  : 판정 과정을 그린 화면(사람 확인용)
    - 로그             : 기동 때 코드 커밋·모델 해시·설정 값

읽는 순서: PARAMS → main() 앞부분(기동 준비) → step()(프레임 1장 처리, 핵심) → on_set_watch()(감시 요청)
ROS·OpenCV·YOLO 는 main() 안에서 import 한다. 그래서 시험은 ROS 없이 이 파일의 함수를 쓸 수 있다.
"""
import hashlib
import subprocess
import time
from pathlib import Path
from types import SimpleNamespace as NS
from uuid import uuid4

from rescue_contracts.clock_measurement import ClockSampler

from .adapter import SourceBinding
from .core import PerceptionError
from .disaster_sign import DisasterSignDetector, DisasterSignPolicy
from .target_watch import TargetWatchPolicy
from .webcam_clock import clock_settings, observation_clock
from .webcam_service import WatchRequest, WatchSlot, boxes_from_xyxyn, scene_visibility, stream_state

ENCODING = 'bgr8'  # OpenCV 프레임의 채널 순서(파랑·초록·빨강)
STAMP_SEMANTICS = 'DRIVER_RECEIVE'  # 프레임 시각 = 카메라에서 받은 직후의 시스템 시각
EVIDENCE_MIME = 'application/octet-stream'  # 증거는 사진 파일이 아니라 원 픽셀 바이트의 해시

# YAML 에서 읽는 값 (이름: 타입). 모두 필수이고 하나라도 빠지면 기동하지 않는다.
# 앞쪽은 카메라·시계·모델·토픽 이름, quake_* 는 지진 판정 값, watch_* 는 출구 판정 값이다.
PARAMS = {
    'camera_id': 'STRING', 'frame_id': 'STRING', 'device_path': 'STRING', 'width': 'INTEGER',
    'height': 'INTEGER', 'frame_period_s': 'DOUBLE', 'reopen_period_s': 'DOUBLE',
    'clock_domain_id': 'STRING',
    # 시계 상태는 OS 시계를 실제로 측정해서 정한다(ClockSampler 가 읽고 webcam_clock 이 메시지 칸으로 바꾼다).
    # '동기화됨'을 설정값으로 적는 칸은 없다.
    'clock_backend': 'STRING', 'clock_max_age_ns': 'INTEGER', 'clock_drift_ppm': 'DOUBLE',
    'clock_poll_period_s': 'DOUBLE',
    'calibration_id': 'STRING', 'observation_validity_s': 'DOUBLE',
    'model_path': 'STRING', 'model_sha256': 'STRING', 'model_imgsz': 'INTEGER', 'model_conf': 'DOUBLE',
    'envelope_schema_version': 'STRING', 'publisher_id': 'STRING', 'policy_version': 'STRING',
    'status_topic': 'STRING', 'site_events_topic': 'STRING', 'presence_topic': 'STRING',
    'set_watch_service': 'STRING',
    'quake_sample_stride': 'INTEGER', 'quake_blocks_x': 'INTEGER', 'quake_blocks_y': 'INTEGER',
    'quake_search_cells': 'INTEGER', 'quake_texture_min': 'DOUBLE', 'quake_match_min': 'DOUBLE',
    'quake_valid_ratio_min': 'DOUBLE', 'quake_agree_tol': 'DOUBLE', 'quake_shift_min': 'DOUBLE',
    'quake_scene_match_min': 'DOUBLE', 'quake_visible_min': 'DOUBLE', 'quake_rebase_s': 'DOUBLE',
    'quake_shakes_min': 'INTEGER', 'quake_window_s': 'DOUBLE', 'quake_hold_s': 'DOUBLE',
    'quake_release_s': 'DOUBLE', 'quake_max_gap_s': 'DOUBLE',
    'watch_score_min': 'DOUBLE', 'watch_roi_overlap_min': 'DOUBLE', 'watch_present_confirm_frames': 'INTEGER',
    'watch_absent_hold_s': 'DOUBLE', 'watch_absent_min_frames': 'INTEGER', 'watch_max_gap_s': 'DOUBLE',
    'watch_exit_roi': 'DOUBLE_ARRAY', 'watch_track_max_jump': 'DOUBLE',
    'debug_image_topic': 'STRING',
}


def _ns(seconds):
    """초 → 나노초(정수). 판정 코드는 시간을 모두 정수 나노초로 다룬다."""
    return int(round(seconds * 1e9))


def git_sha():
    """지금 돌고 있는 코드의 git 커밋. 기동 로그에 남겨 '어느 코드로 실험했나'를 나중에 확인한다."""
    try:
        out = subprocess.run(['git', '-C', str(Path(__file__).resolve().parent), 'rev-parse', 'HEAD'],
                             capture_output=True, text=True, timeout=5, check=True)
        return out.stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return 'UNKNOWN'


def file_sha256(path):
    """파일 해시(sha256). 모델 파일이 YAML 에 적힌 그 파일이 맞는지 확인하는 데 쓴다."""
    digest = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):  # 1 MB 씩 읽는다
            digest.update(chunk)
    return digest.hexdigest()


def policies(v):
    """YAML 값 → 지진 판정 설정(DisasterSignPolicy), 출구 판정 설정(TargetWatchPolicy).

    초 단위 값(*_s)은 나노초로 바꿔 넣는다. 값이 이상하면 각 설정 클래스가 오류를 낸다.
    """
    quake = DisasterSignPolicy(
        sample_stride=v['quake_sample_stride'], blocks_x=v['quake_blocks_x'], blocks_y=v['quake_blocks_y'],
        search_cells=v['quake_search_cells'], texture_min=v['quake_texture_min'], match_min=v['quake_match_min'],
        valid_ratio_min=v['quake_valid_ratio_min'], agree_tol=v['quake_agree_tol'], shift_min=v['quake_shift_min'],
        scene_match_min=v['quake_scene_match_min'], visible_min=v['quake_visible_min'],
        rebase_ns=_ns(v['quake_rebase_s']), shakes_min=v['quake_shakes_min'], window_ns=_ns(v['quake_window_s']),
        hold_ns=_ns(v['quake_hold_s']), release_ns=_ns(v['quake_release_s']), max_gap_ns=_ns(v['quake_max_gap_s']))
    watch = TargetWatchPolicy(
        score_min=v['watch_score_min'], roi_overlap_min=v['watch_roi_overlap_min'],
        present_confirm_frames=v['watch_present_confirm_frames'], absent_hold_ns=_ns(v['watch_absent_hold_s']),
        absent_min_frames=v['watch_absent_min_frames'], max_gap_ns=_ns(v['watch_max_gap_s']),
        exit_roi_xyxy_normalized=tuple(float(x) for x in v['watch_exit_roi']),
        track_max_jump=v['watch_track_max_jump'])
    return quake, watch


def spin_until_shutdown(rclpy, node, log):
    """노드를 돌린다. Ctrl+C 로 끄면 정상 종료로 처리한다.

    Ctrl+C 가 프레임 처리 도중에 오면, 처리 중이던 발행이 '이미 종료됨' 오류를 낸다. 이것도 정상 종료로 본다.
    종료 중이 아닌데 난 오류는 버그이므로 그대로 올린다.
    """
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    log.info('webcam_node stopping')


def frame_image(frame, stamp_ns, frame_id):
    """OpenCV 프레임(세로×가로×3 바이트 배열) → 판정 코드가 읽는 이미지 모양.

    판정 코드는 ROS Image 메시지와 같은 이름의 칸(header·width·height·encoding·step·data)을 읽는다.
    ROS 메시지를 만들지 않고 이름만 같은 객체로 넘겨서, 시험에서도 ROS 없이 같은 입력을 만들 수 있다.
    """
    height, width = frame.shape[:2]
    return NS(header=NS(stamp=NS(sec=stamp_ns // 10**9, nanosec=stamp_ns % 10**9), frame_id=frame_id),
              width=width, height=height, encoding=ENCODING, is_bigendian=0, step=width * 3, data=frame.tobytes())


def device_change_refusal(name, value):
    """실행 중에 파라미터를 바꾸려 할 때의 거절 이유. 받아들이면 None.

    관제 웹에서 웹캠을 고를 때 쓴다. 바꿀 수 있는 것은 device_path 하나뿐이다.
    없는 장치 경로면 거절한다. 조용히 이전 장치로 두지 않고, 거절 이유가 관제 웹까지 전달된다.
    """
    if name != 'device_path':
        return f'{name} is fixed at start; only device_path can change'
    if not isinstance(value, str) or not value:
        return 'device_path must be a device path'
    if not Path(value).exists():
        return f'no such camera device: {value}'
    return None


def main(args=None):
    import cv2
    import rclpy
    from rclpy.exceptions import ParameterUninitializedException
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rescue_interfaces.msg import CameraStatus, SiteEvent, TargetPresence
    from sensor_msgs.msg import CompressedImage
    from rescue_interfaces.srv import SetCameraWatch
    from ultralytics import YOLO

    rclpy.init(args=args)
    node = Node('webcam_node')
    log = node.get_logger()

    # ── 기동 준비 ①: 파라미터 읽기. 빠진 값은 모아서 한 번에 알리고 종료한다(exit 2).
    values = {}
    for name, kind in PARAMS.items():
        # 기본값 없이 선언 → YAML 에 없으면 값을 읽을 때 오류가 나서 None 으로 둔다
        node.declare_parameter(name, getattr(Parameter.Type, kind))
        try:
            values[name] = node.get_parameter(name).value
        except ParameterUninitializedException:
            values[name] = None
    missing = sorted(name for name, value in values.items() if value is None)
    if missing:
        log.fatal(f'missing parameters: {missing}')
        raise SystemExit(2)
    v = values
    boot_id = str(uuid4())  # 이번 실행의 ID. 노드를 다시 켜면 바뀌어서 받는 쪽이 재시작을 알 수 있다

    # ── 기동 준비 ②: 값 검사. 판정 값이 범위를 벗어나거나 시계 설정이 틀리면 기동하지 않는다.
    try:
        quake_policy, watch_policy = policies(v)
        # binding = 이 카메라의 신원(카메라 ID·실행 ID 등). 판정기가 사건을 만들 때 넣는다.
        # 시계 상태는 여기서 쓰지 않는다. 메시지를 보낼 때 obs_time() 이 측정값으로 채운다.
        binding = SourceBinding(None, v['camera_id'], boot_id, v['clock_domain_id'], STAMP_SEMANTICS, 'UNKNOWN',
                                v['calibration_id'], 'UNKNOWN', None)
    except PerceptionError as exc:
        log.fatal(f'invalid parameters: {exc.detail_code}: {exc.detail}')
        raise SystemExit(2) from exc
    try:
        clock_backend, clock_max_age_ns, clock_drift_ppm = clock_settings(v)
        if not v['clock_poll_period_s'] > 0:
            raise ValueError('clock_poll_period_s must be positive')
    except ValueError as exc:
        log.fatal(f'invalid parameters: {exc}')
        raise SystemExit(2) from exc

    # ── 기동 준비 ③: 모델 확인. 파일 해시가 YAML 값과 다르면 다른 가중치이므로 기동하지 않는다.
    model_sha = file_sha256(v['model_path'])
    if model_sha != v['model_sha256']:
        log.fatal(f'model sha256 mismatch: file={model_sha} expected={v["model_sha256"]}')
        raise SystemExit(2)
    model = YOLO(v['model_path'])

    # ── 기동 준비 ④: 판정기 만들기
    #   quake : 지진 판정기. 노드가 켜져 있는 동안 매 프레임 돈다.
    #   slot  : 감시 칸. 감시가 걸리면 slot.detector(출구 판정기)가 생기고, 그때부터 YOLO 를 돌린다.
    quake = DisasterSignDetector(binding, quake_policy)
    slot = WatchSlot(binding, watch_policy)
    validity_ns = _ns(v['observation_validity_s'])  # 메시지 유효 시간(예: 1초). 지난 메시지는 받는 쪽이 쓰지 않는다

    # 시계 측정: ClockSampler 가 OS 시계 서비스(timesyncd·chrony)를 주기적으로 읽어 둔다.
    # 메시지마다 '이 시각을 믿을 수 있나'(SYNCED/UNKNOWN)를 싣는다.
    # 판정 결과에는 영향이 없다.
    sampler = (ClockSampler(clock_backend, clock_max_age_ns, clock_drift_ppm) if clock_backend != 'NONE' else None)
    clock = {'record': dict(state='UNKNOWN', reason='clock_backend NONE' if sampler is None else 'waiting'),
             'logged': None}

    def poll_clock():
        """시계 측정값 갱신. 상태가 바뀔 때만 로그를 남긴다."""
        clock['record'] = sampler.poll()
        record = clock['record']
        shown = (record.get('state'), record.get('reason'))
        if shown != clock['logged']:
            clock['logged'] = shown
            if record.get('state') == 'SYNCED':
                log.info(f'clock SYNCED backend={clock_backend} error_bound_ns={record["error_bound_ns"]} '
                         f'measured_at_ns={record["measured_at_ns"]}')
            else:
                log.warning(f'clock UNKNOWN backend={clock_backend}: {record.get("reason")}')

    # ── 기동 준비 ⑤: 기동 로그. 코드 커밋·모델 해시·실제 장치·설정 값을 남겨 나중에 같은 조건을 재현할 수 있게 한다.
    log.info(f'webcam_node git={git_sha()} boot={boot_id} model={v["model_path"]} sha256={model_sha} '
             f'camera={v["camera_id"]} device={v["device_path"]}->{Path(v["device_path"]).resolve()} {v["width"]}x{v["height"]} '
             f'period={v["frame_period_s"]}s policy={v["policy_version"]} quake={quake_policy} watch={watch_policy} '
             f'clock={clock_backend} domain={v["clock_domain_id"]} max_age_ns={clock_max_age_ns} '
             f'drift_ppm={clock_drift_ppm} poll={v["clock_poll_period_s"]}s')

    # ── 발행자(나가는 토픽)
    #   status      : 최신 상태만 의미가 있어 1개만 보관
    #   site_events : 사건은 놓치면 안 되므로 10개까지 보관
    #   presence    : TRANSIENT_LOCAL = 나중에 구독을 시작한 노드도 마지막 값을 바로 받는다
    #   debug_image : 화면 확인용이라 끊겨도 되는 BEST_EFFORT. 토픽 이름을 비우면 아예 만들지 않는다
    reliable = dict(reliability=ReliabilityPolicy.RELIABLE)
    status_pub = node.create_publisher(CameraStatus, v['status_topic'],
                                       QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE, **reliable))
    event_pub = node.create_publisher(SiteEvent, v['site_events_topic'],
                                      QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE, **reliable))
    presence_pub = node.create_publisher(TargetPresence, v['presence_topic'],
                                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, **reliable))
    seq = {'status': 0, 'event': 0, 'presence': 0, 'frame': 0, 'srv': 0}  # 메시지 종류별 일련번호, 'frame' = 읽은 프레임 수
    debug_pub = (node.create_publisher(CompressedImage, v['debug_image_topic'],
                                       QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
                 if v['debug_image_topic'] else None)
    trail = {'detector': None, 'points': []}  # 판정 화면에 그릴 인형 이동 경로(최근 25점)

    def label(img, text, org, color):
        """어두운 바탕 상자 위에 글자를 쓴다(작은 글씨도 읽히게)."""
        (tw, th), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        x, y = org
        cv2.rectangle(img, (x - 4, y - th - 5), (x + tw + 4, y + base + 2), (20, 20, 20), -1)
        cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

    def publish_debug(frame, stamp, sign, boxes, presence):
        """판정 화면(JPEG) 발행. 판정에는 쓰지 않고 사람이 눈으로 확인하는 용도다.

        흰 격자 = 지진 판정 블록(굵은 선 = 사분면 경계), 노란 화살표 = 화면 이동, 빨간 영역 = 출구,
        초록 상자 = 감시 영역(화면 전체면 안 그림), 파란 상자 = 인형(점수 통과), 회색 상자 = 점수 미달,
        노란 선·점 = 추적 경로. 왼쪽 위 글자 = 방향 전환 횟수(지진 조건이 참이면 보라), 아래 글자 = 출구 판정 상태.
        """
        if debug_pub is None:
            return
        img = frame.copy()
        height, width = img.shape[:2]

        def px(x, y):
            return int(x * width), int(y * height)  # 화면 비율 좌표 → 픽셀 좌표
        grid = img.copy()
        bx, by = v['quake_blocks_x'], v['quake_blocks_y']
        for i in range(1, bx):
            cv2.line(grid, (width * i // bx, 0), (width * i // bx, height), (255, 255, 255), 2 if i * 2 == bx else 1)
        for j in range(1, by):
            cv2.line(grid, (0, height * j // by), (width, height * j // by), (255, 255, 255), 2 if j * 2 == by else 1)
        img = cv2.addWeighted(grid, 0.22, img, 0.78, 0)  # 격자를 반투명하게 겹친다
        if sign.shift is not None and any(sign.shift):
            scale = v['quake_sample_stride'] * 6   # 잘 보이게 실제 이동보다 크게 그린다
            centre = (width // 2, height // 2)
            end = (int(centre[0] + sign.shift[0] * scale), int(centre[1] + sign.shift[1] * scale))
            cv2.arrowedLine(img, centre, end, (0, 0, 0), 9, cv2.LINE_AA, tipLength=0.3)
            cv2.arrowedLine(img, centre, end, (10, 214, 255), 5, cv2.LINE_AA, tipLength=0.3)
        quake = f'QUAKE turns {sign.reversals}/{v["quake_shakes_min"]}' + (f' {sign.reason}' if sign.reason else '')
        ex = watch_policy.exit_roi_xyxy_normalized
        tint = img.copy()
        cv2.rectangle(tint, px(ex[0], ex[1]), px(ex[2], ex[3]), (48, 59, 255), -1)
        img = cv2.addWeighted(tint, 0.2, img, 0.8, 0)
        cv2.rectangle(img, px(ex[0], ex[1]), px(ex[2], ex[3]), (48, 59, 255), 2)
        watch = slot.watch
        if watch is not None and tuple(watch.roi_xyxy_normalized) != (0.0, 0.0, 1.0, 1.0):
            r = watch.roi_xyxy_normalized
            cv2.rectangle(img, px(r[0], r[1]), px(r[2], r[3]), (89, 199, 52), 2)
        if trail['detector'] is not slot.detector:  # 감시가 새로 걸렸으면 경로를 비운다
            trail['detector'], trail['points'] = slot.detector, []
        track = slot.detector.track if slot.detector is not None else None
        if track is not None and (not trail['points'] or trail['points'][-1] != track):
            trail['points'] = (trail['points'] + [track])[-25:]
        for a, b in zip(trail['points'], trail['points'][1:]):
            cv2.line(img, px(*a), px(*b), (10, 214, 255), 2, cv2.LINE_AA)
        for box in boxes or ():
            color = (255, 132, 10) if box.score >= watch_policy.score_min else (147, 142, 142)
            cv2.rectangle(img, px(box.x0, box.y0), px(box.x1, box.y1), color, 2, cv2.LINE_AA)
            cv2.putText(img, f'{box.score:.2f}', px(box.x0, box.y0 - 0.01), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        if track is not None:
            cv2.circle(img, px(*track), 8, (10, 214, 255), -1, cv2.LINE_AA)
        if presence is not None:
            text = (f'{presence.state} {presence.reason or ""} absent={presence.absent_ns / 1e9:.1f}s '
                    f'rev={watch.watch_revision if watch else 0}')
        else:
            text = 'NO WATCH'
        label(img, quake, (10, 22), (242, 90, 191) if sign.condition else (255, 255, 255))
        label(img, text, (10, height - 10), (255, 255, 255))
        ok, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            log.error('debug image encode failed')
            return
        msg = CompressedImage()
        msg.header.stamp.sec, msg.header.stamp.nanosec = stamp // 10**9, stamp % 10**9
        msg.header.frame_id = v['frame_id']
        msg.format = 'jpeg'
        msg.data = jpeg.tobytes()
        debug_pub.publish(msg)

    # ── 모든 메시지에 들어가는 공통 칸 채우기
    def envelope(env, key):
        """누가(publisher_id·boot_id), 몇 번째(sequence), 언제(published_at) 보냈나."""
        seq[key] += 1
        env.schema_version = v['envelope_schema_version']
        env.publisher_id = v['publisher_id']
        env.publisher_boot_id = boot_id
        env.sequence = seq[key]
        env.published_at_ns = time.time_ns()

    def obs_time(t, observed_ns):
        """언제 찍은 프레임인지, 언제까지 유효한지, 그 시각을 믿을 수 있는지(시계 상태)."""
        t.observed_at_ns = observed_ns
        t.valid_until_ns = observed_ns + validity_ns
        t.stamp_semantics = STAMP_SEMANTICS
        t.clock_domain_id = v['clock_domain_id']
        t.clock_state, error_bound = observation_clock(clock['record'], observed_ns)
        t.has_clock_error_bound = error_bound is not None
        t.clock_error_bound_ns = error_bound or 0

    def evidence(ref, evidence_id, sha256, observed_ns):
        """근거 프레임 정보. sha256 은 그 프레임 원 픽셀의 해시다(사진 파일 업로드는 아직 없음)."""
        ref.evidence_id = evidence_id
        ref.sha256 = sha256
        ref.mime_type = EVIDENCE_MIME
        ref.source_id = v['camera_id']
        ref.source_boot_id = boot_id
        ref.observed_at_ns = observed_ns

    def publish_status(state, observed_ns, width=0, height=0, reason=''):
        """CameraStatus 발행: 카메라 상태(ONLINE / OFFLINE / DEGRADED / UNKNOWN)와 이유."""
        msg = CameraStatus()
        envelope(msg.envelope, 'status')
        msg.camera_id = v['camera_id']
        msg.source_boot_id = boot_id
        obs_time(msg.time, observed_ns)
        msg.stream_state = state
        msg.image_width, msg.image_height = width, height
        msg.frame_sequence = seq['frame']
        msg.quality_reason = reason
        status_pub.publish(msg)

    def publish_event(event):
        """SiteEvent 발행: 판정기가 만든 사건(지진 또는 구조 후보)을 메시지로 옮긴다.

        사건은 화면이 잘 보일 때(VALID)만 생기므로 visibility_state 는 항상 VALID 다.
        """
        msg = SiteEvent()
        envelope(msg.envelope, 'event')
        msg.event_id = event.event_id
        msg.camera_id = event.camera_id
        msg.source_boot_id = boot_id
        msg.kind = event.kind
        obs_time(msg.time, event.observed_at_ns)
        msg.target_id = event.target_id or ''
        msg.watch_id = event.watch_id or ''
        msg.confidence = float(event.confidence)
        msg.visibility_state = 'VALID'
        msg.policy_version = v['policy_version']
        evidence(msg.evidence, event.evidence_id, event.evidence_sha256, event.observed_at_ns)
        event_pub.publish(msg)
        log.info(f'SiteEvent {event.kind} id={event.event_id} watch={event.watch_id} detail={event.detail}')

    def publish_presence(watch, result, observed_ns, sha256):
        """TargetPresence 발행: 감시 중일 때 프레임마다 인형 상태(PRESENT / ABSENT_CANDIDATE / UNKNOWN)와 시야."""
        msg = TargetPresence()
        envelope(msg.envelope, 'presence')
        msg.camera_id = watch.camera_id
        msg.watch_id = watch.watch_id
        msg.target_id = watch.target_id
        msg.watch_revision = watch.watch_revision
        obs_time(msg.time, observed_ns)
        msg.roi_xyxy_normalized = list(watch.roi_xyxy_normalized)
        msg.state = result.state or 'UNKNOWN'
        msg.visibility_state = result.visibility_state or 'UNKNOWN'
        msg.consecutive_valid_frames = result.consecutive_valid_frames
        evidence(msg.evidence, f'{v["camera_id"]}/{observed_ns}', sha256, observed_ns)
        presence_pub.publish(msg)

    def on_set_watch(request, response):
        """SetCameraWatch 요청 처리(감시 걸기·해제).

        ROS 요청을 WatchRequest 로 바꿔 slot.handle() 에 넘기고, 결과를 응답에 옮긴다. 규칙은 webcam_service.WatchSlot.
        요청 형식이 틀려도 노드는 죽지 않고 INVALID_INPUT 으로 답한다.
        """
        try:
            req = WatchRequest(
                command_id=request.command_id, camera_id=request.camera_id, watch_id=request.watch_id,
                target_id=request.target_id, expected_watch_revision=int(request.expected_watch_revision),
                roi_xyxy_normalized=tuple(float(x) for x in request.roi_xyxy_normalized),
                association_method=request.association_method,
                association_evidence_id=request.association_evidence_id, enable=bool(request.enable),
                issued_at_ns=int(request.issued_at_ns), deadline_ns=int(request.deadline_ns))
            # 수락·거절 판단 (지금 시각으로 기한을 확인)
            reply = slot.handle(req, now_ns=time.time_ns())
        # 요청 칸 형식이 틀려 WatchRequest 를 못 만들면 여기서 INVALID_INPUT 응답을 만든다
        except (PerceptionError, TypeError, ValueError) as exc:
            log.error(f'SetCameraWatch rejected (bad request): {exc}')
            reply = NS(accepted=False, watch_id=request.watch_id, watch_revision=slot.revision,
                       reason_code='INVALID_INPUT', detail=str(exc))
        # 응답 메시지 채우기
        envelope(response.envelope, 'srv')
        response.accepted = reply.accepted
        response.watch_id = reply.watch_id
        response.watch_revision = reply.watch_revision
        response.reason_code = reply.reason_code
        log.info(f'SetCameraWatch command={request.command_id} enable={request.enable} '
                 f'watch={request.watch_id} -> {reply.reason_code} revision={reply.watch_revision}'
                 + (f' detail={reply.detail}' if reply.detail else ''))
        return response

    node.create_service(SetCameraWatch, v['set_watch_service'], on_set_watch)

    # 관제 웹 카메라 고르기(OPS-183): device_path 를 실행 중에 바꾼다. 다음 틱에 카메라를 닫고 새 장치로 열며,
    # 지진 판정을 처음부터 다시 하고(화면이 바뀜) 출구 감시를 푼다(ROI 가 다른 화면을 가리킴).
    switch = {'to': None}

    def on_set_parameters(params):
        """파라미터 바꾸기 요청 처리. 하나라도 거절이면 전부 거절, 모두 통과하면 바꿀 장치를 switch['to'] 에 적어 둔다.

        실제로 카메라를 바꾸는 일은 다음 step() 에서 한다(타이머 콜백과 같은 스레드에서 안전하게).
        """
        from rcl_interfaces.msg import SetParametersResult
        for param in params:
            refusal = device_change_refusal(param.name, param.value)
            if refusal is not None:
                log.warning(f'parameter change refused: {refusal}')
                return SetParametersResult(successful=False, reason=refusal)
        switch['to'] = params[-1].value if params else None
        return SetParametersResult(successful=True)

    node.add_on_set_parameters_callback(on_set_parameters)

    camera = {'cap': None, 'next_open_ns': 0}  # cap = 열린 카메라(없으면 None), next_open_ns = 다음에 다시 열어 볼 시각

    def open_camera():
        """카메라 열기. 실패하면 None.

        장치 번호(/dev/video2) 대신 고정 경로(/dev/v4l/by-id/...)로 연다. 번호는 USB 를 꽂는 순서에 따라 바뀐다.
        """
        cap = cv2.VideoCapture(v['device_path'], cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, v['width'])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, v['height'])
        # 0.2초마다 한 장만 읽으므로, 그 사이 쌓인 예전 프레임이 나오지 않게 버퍼를 1장으로 둔다(지원 안 하면 무시됨).
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            cap.release()
            return None
        return cap

    def step():
        """★ 핵심: 타이머가 0.2초마다 부른다. 프레임 1장을 읽어 두 판정에 넣고 결과를 발행한다.

        (0) 카메라 바꾸기 요청 처리 → (1) 카메라 열기 → (2) 프레임 읽기 → (3) 크기 확인 → (4) 지진 판정 → (5) 카메라 상태·지진 사건 발행
        → (6) 감시가 없으면 여기서 끝 → (7) YOLO 로 인형 찾기 → (8) 시야 정하기 → (9) 출구 판정 → (10) 결과 발행
        """
        # 카메라를 바꿀 때 quake 를 새 판정기로 바꿔 넣으려고 바깥 변수로 선언
        nonlocal quake
        now = time.time_ns()
        # (0) 카메라 바꾸기 요청이 있으면: 지금 카메라를 닫고, 지진 판정을 처음부터 다시 하고(화면이 바뀜),
        #     감시를 푼 뒤(감시 영역이 다른 화면을 가리킴) OFFLINE / CAMERA_SWITCHED 를 보낸다. 새 카메라는 다음 틱 (1) 에서 연다.
        if switch['to'] is not None:
            old, v['device_path'], switch['to'] = v['device_path'], switch['to'], None
            if camera['cap'] is not None:
                camera['cap'].release()
            camera['cap'], camera['next_open_ns'] = None, 0
            quake = DisasterSignDetector(binding, quake_policy)
            slot.camera_changed()
            publish_status('OFFLINE', now, reason='CAMERA_SWITCHED')
            log.warning(f'CAMERA_SWITCHED {old} -> {v["device_path"]}: quake judge restarted, watch cleared '
                        f'(revision {slot.revision})')
            return
        # (1) 카메라가 닫혀 있으면 다시 연다. 실패하면 OFFLINE 을 보내고 2초 뒤에 다시 시도한다.
        if camera['cap'] is None:
            if now < camera['next_open_ns']:
                return
            camera['cap'] = open_camera()
            if camera['cap'] is None:
                camera['next_open_ns'] = now + _ns(v['reopen_period_s'])
                publish_status('OFFLINE', now, reason='OPEN_FAILED')
                log.warning(f'camera {v["device_path"]} open failed; retry in {v["reopen_period_s"]}s')
                return
        # (2) 프레임 읽기. 시각은 받은 직후의 시스템 시각이다.
        #     실패하면(USB 빠짐 등) 카메라를 닫고 OFFLINE 을 보낸다. 다음 틱에 (1) 에서 다시 연다.
        ok, frame = camera['cap'].read()
        stamp = time.time_ns()
        # 읽기 실패이거나 컬러 3채널 영상이 아니면 실패로 본다
        if not ok or frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            camera['cap'].release()
            camera['cap'] = None
            publish_status('OFFLINE', stamp, reason='READ_FAILED')
            log.warning('camera read failed; reopening')
            return
        # (3) 영상 크기가 바뀌면 감시를 해제한다(감시 영역이 다른 곳을 가리키게 되므로).
        seq['frame'] += 1
        # frame.shape = (세로, 가로, 3)
        height, width = frame.shape[:2]
        if slot.observe_image_size(width, height):
            log.warning(f'image size changed to {width}x{height}; watch cleared (revision {slot.revision})')
        image = frame_image(frame, stamp, v['frame_id'])  # 두 판정 모두 이 image(같은 프레임·같은 시각)를 쓴다
        # (4) 지진 판정 → sign (SignFrame: 보류 여부와 이유, 화면 이동, 방향 전환 횟수, 사건)
        #     판정 중 오류가 나면 카메라 상태 UNKNOWN 을 보내고 이번 프레임은 건너뛴다.
        try:
            sign = quake.update(image)
        except PerceptionError as exc:
            publish_status('UNKNOWN', stamp, width, height, reason='QUAKE_JUDGE_FAILED')
            log.error(f'quake judge failed: {exc.detail_code}: {exc.detail}')
            return
        # (5) 카메라 상태 발행(해상도 다름 → DEGRADED, 화면 가림 → UNKNOWN, 아니면 ONLINE). 지진 사건이 있으면 발행.
        state, reason = stream_state(sign, (width, height), (v['width'], v['height']))
        publish_status(state, stamp, width, height, reason=reason)
        if sign.event is not None:
            publish_event(sign.event)
        # (6) 감시가 없으면(평상시) 판정 화면만 보내고 끝. YOLO 는 감시가 걸렸을 때만 돌려 CPU 를 아낀다.
        if slot.detector is None:
            publish_debug(frame, stamp, sign, None, None)
            return
        # (7) YOLO 로 인형 상자 찾기. 좌표는 화면 비율(0 ~ 1)로 받는다.
        #     model_conf(0.1)는 YOLO 출력 하한일 뿐이고, 인형으로 인정하는 점수(0.25)는 출구 판정기가 따로 거른다.
        #     YOLO 가 실패하면 boxes = None → 출구 판정기가 '판정 불가'로 처리한다(사라진 것으로 세지 않음).
        try:
            result = model.predict(frame, imgsz=v['model_imgsz'], conf=v['model_conf'], verbose=False)[0]
            boxes = boxes_from_xyxyn(result.boxes.xyxyn.tolist(), result.boxes.conf.tolist())
        except Exception as exc:
            log.error(f'detector failed: {exc}')
            boxes = None
        # (8) 시야: 가림을 따로 재지 않고 지진 판정 결과로 정한다(VALID / OCCLUDED / UNKNOWN).
        visibility = scene_visibility(sign, stream_online=True)
        watch = slot.watch
        # (9) 출구 구조 판정 → presence (PresenceFrame: 인형 상태, 이유, 사라진 시간, 후보 사건)
        try:
            presence = slot.detector.update(image, boxes, visibility=visibility)
        except PerceptionError as exc:
            log.error(f'watch judge failed: {exc.detail_code}: {exc.detail}')
            return
        if presence.status != 'PROCESSED':  # 같은·이른 시각 프레임이라 판정하지 않았다
            return
        # (10) 판정 화면과 인형 상태를 발행하고, 구조 후보 사건이 있으면 발행한다.
        publish_debug(frame, stamp, sign, boxes, presence)
        # 이 프레임 원 픽셀의 해시 = presence 메시지의 증거
        sha256 = hashlib.sha256(image.data).hexdigest()
        publish_presence(watch, presence, stamp, sha256)
        if presence.event is not None:
            publish_event(presence.event)

    # 타이머 등록. 콜백은 한 스레드에서 차례로 돌아서, 한 프레임 처리가 0.2초보다 길면 다음 프레임이 밀린다.
    node.create_timer(v['frame_period_s'], step)
    if sampler is not None:
        poll_clock()
        node.create_timer(v['clock_poll_period_s'], poll_clock)
    else:
        log.warning('clock_backend NONE: every message is clock_state UNKNOWN')
    try:
        spin_until_shutdown(rclpy, node, log)
    finally:
        # 종료할 때 시계 측정·카메라·노드를 차례로 닫는다.
        if sampler is not None:
            sampler.close()
        if camera['cap'] is not None:
            camera['cap'].release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
