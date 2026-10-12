"""ROS 2 경계: 고정 웹캠 1대 -> CameraStatus·SiteEvent·TargetPresence, SetCameraWatch 서비스.

SDD v0.1 의 cam.capture·cam.quake·cam.presence(IF-075 ~ IF-078)를 PC3 의 프로세스 하나로 묶는다.
프레임 하나를 읽어 지진 판정(``DisasterSignDetector``)과 인형 탐지(웹캠 survivor 모델) -> 소실 판정
(``TargetWatchDetector``)에 같은 프레임·같은 시각으로 넣는다. 원 영상 토픽(IF-074)과 영상
게이트웨이(cam.video)는 이 노드에 없다.

- 판정 규칙은 ``disaster_sign``·``target_watch``·``webcam_service`` 에 있고 이 파일은 옮기기만 한다.
- 시각: 프레임을 받은 직후의 system time 으로 찍는다(stamp_semantics=DRIVER_RECEIVE).
- 사건은 VALID 시야에서만 생긴다(C-10 경계). 판정 보류는 SiteEvent 로 보내지 않는다.
- 증거: SiteEvent·TargetPresence 의 sha256 은 그 프레임 원 픽셀(bgr8) 바이트의 해시다. 사진 바이트를
  operations 로 올리는 일은 아직 없다(후속).
- 모든 값은 파라미터로 받는다(기본값 없음, AGENTS 8절). 모델 파일의 sha256 이 파라미터와 다르면
  기동하지 않는다. 기동 때 git SHA·모델 해시·적용 설정을 로그에 남긴다.

ROS·OpenCV·ultralytics 는 main() 안에서 import 한다(이 모듈은 ROS 없이 import 된다).
"""
import hashlib
import subprocess
import time
from pathlib import Path
from types import SimpleNamespace as NS
from uuid import uuid4

from .adapter import SourceBinding
from .core import PerceptionError
from .disaster_sign import DisasterSignDetector, DisasterSignPolicy
from .target_watch import TargetWatchPolicy
from .webcam_service import WatchRequest, WatchSlot, boxes_from_xyxyn, scene_visibility

ENCODING = 'bgr8'  # OpenCV 프레임 채널 순서
STAMP_SEMANTICS = 'DRIVER_RECEIVE'
EVIDENCE_MIME = 'application/octet-stream'  # 원 픽셀 바이트(bgr8)의 해시. 사진 파일이 아니다.

# (이름, 타입). 모두 필수이고 기본값이 없다.
PARAMS = {
    'camera_id': 'STRING', 'frame_id': 'STRING', 'device_path': 'STRING', 'width': 'INTEGER',
    'height': 'INTEGER', 'frame_period_s': 'DOUBLE', 'reopen_period_s': 'DOUBLE',
    'clock_domain_id': 'STRING', 'clock_state': 'STRING', 'clock_error_bound_ns': 'INTEGER',
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
}


def _ns(seconds):
    return int(round(seconds * 1e9))


def git_sha():
    try:
        out = subprocess.run(['git', '-C', str(Path(__file__).resolve().parent), 'rev-parse', 'HEAD'],
                             capture_output=True, text=True, timeout=5, check=True)
        return out.stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return 'UNKNOWN'


def file_sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            digest.update(chunk)
    return digest.hexdigest()


def policies(v):
    """파라미터 -> (DisasterSignPolicy, TargetWatchPolicy). 값 검사는 각 정책이 한다."""
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
        absent_min_frames=v['watch_absent_min_frames'], max_gap_ns=_ns(v['watch_max_gap_s']))
    return quake, watch


def spin_until_shutdown(rclpy, node, log):
    """spin. 종료(Ctrl+C·외부 shutdown)와 종료 뒤 처리 중이던 콜백의 오류는 정상 종료로 본다.

    SIGINT 가 프레임 처리 도중에 오면 그 콜백의 발행이 RCLError(context invalid)를 낸다(2026-10-12 실기).
    context 가 살아 있는데 난 오류는 버그이므로 그대로 올린다.
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
    """OpenCV 프레임(H×W×3 bgr8) -> adapter 가 읽는 Image 모양 객체."""
    height, width = frame.shape[:2]
    return NS(header=NS(stamp=NS(sec=stamp_ns // 10**9, nanosec=stamp_ns % 10**9), frame_id=frame_id),
              width=width, height=height, encoding=ENCODING, is_bigendian=0, step=width * 3, data=frame.tobytes())


def main(args=None):
    import cv2
    import rclpy
    from rclpy.exceptions import ParameterUninitializedException
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rescue_interfaces.msg import CameraStatus, SiteEvent, TargetPresence
    from rescue_interfaces.srv import SetCameraWatch
    from ultralytics import YOLO

    rclpy.init(args=args)
    node = Node('webcam_node')
    log = node.get_logger()
    values = {}
    for name, kind in PARAMS.items():
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
    boot_id = str(uuid4())
    try:
        quake_policy, watch_policy = policies(v)
        error_bound = v['clock_error_bound_ns'] if v['clock_error_bound_ns'] >= 0 else None  # -1 = 모름
        binding = SourceBinding(None, v['camera_id'], boot_id, v['clock_domain_id'], STAMP_SEMANTICS, 'UNKNOWN',
                                v['calibration_id'], v['clock_state'], error_bound)
    except PerceptionError as exc:
        log.fatal(f'invalid parameters: {exc.detail_code}: {exc.detail}')
        raise SystemExit(2) from exc
    model_sha = file_sha256(v['model_path'])
    if model_sha != v['model_sha256']:
        log.fatal(f'model sha256 mismatch: file={model_sha} expected={v["model_sha256"]}')
        raise SystemExit(2)
    model = YOLO(v['model_path'])
    quake = DisasterSignDetector(binding, quake_policy)
    slot = WatchSlot(binding, watch_policy)
    validity_ns = _ns(v['observation_validity_s'])
    log.info(f'webcam_node git={git_sha()} boot={boot_id} model={v["model_path"]} sha256={model_sha} '
             f'camera={v["camera_id"]} device={v["device_path"]}->{Path(v["device_path"]).resolve()} {v["width"]}x{v["height"]} '
             f'period={v["frame_period_s"]}s policy={v["policy_version"]} quake={quake_policy} watch={watch_policy}')

    reliable = dict(reliability=ReliabilityPolicy.RELIABLE)
    status_pub = node.create_publisher(CameraStatus, v['status_topic'],
                                       QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE, **reliable))
    event_pub = node.create_publisher(SiteEvent, v['site_events_topic'],
                                      QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE, **reliable))
    presence_pub = node.create_publisher(TargetPresence, v['presence_topic'],
                                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, **reliable))
    seq = {'status': 0, 'event': 0, 'presence': 0, 'frame': 0, 'srv': 0}

    def envelope(env, key):
        seq[key] += 1
        env.schema_version = v['envelope_schema_version']
        env.publisher_id = v['publisher_id']
        env.publisher_boot_id = boot_id
        env.sequence = seq[key]
        env.published_at_ns = time.time_ns()

    def obs_time(t, observed_ns):
        t.observed_at_ns = observed_ns
        t.valid_until_ns = observed_ns + validity_ns
        t.stamp_semantics = STAMP_SEMANTICS
        t.clock_domain_id = v['clock_domain_id']
        t.clock_state = v['clock_state']
        t.has_clock_error_bound = error_bound is not None
        t.clock_error_bound_ns = error_bound or 0

    def evidence(ref, evidence_id, sha256, observed_ns):
        ref.evidence_id = evidence_id
        ref.sha256 = sha256
        ref.mime_type = EVIDENCE_MIME
        ref.source_id = v['camera_id']
        ref.source_boot_id = boot_id
        ref.observed_at_ns = observed_ns

    def publish_status(state, observed_ns, width=0, height=0, reason=''):
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
        try:
            req = WatchRequest(
                command_id=request.command_id, camera_id=request.camera_id, watch_id=request.watch_id,
                target_id=request.target_id, expected_watch_revision=int(request.expected_watch_revision),
                roi_xyxy_normalized=tuple(float(x) for x in request.roi_xyxy_normalized),
                association_method=request.association_method,
                association_evidence_id=request.association_evidence_id, enable=bool(request.enable),
                issued_at_ns=int(request.issued_at_ns), deadline_ns=int(request.deadline_ns))
            reply = slot.handle(req, now_ns=time.time_ns())
        except (PerceptionError, TypeError, ValueError) as exc:
            log.error(f'SetCameraWatch rejected (bad request): {exc}')
            reply = NS(accepted=False, watch_id=request.watch_id, watch_revision=slot.revision,
                       reason_code='INVALID_INPUT', detail=str(exc))
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

    camera = {'cap': None, 'next_open_ns': 0}

    def open_camera():
        cap = cv2.VideoCapture(v['device_path'], cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, v['width'])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, v['height'])
        # 주기 읽기(frame_period_s)라 버퍼에 쌓인 예전 프레임이 나오지 않게 1장만 둔다(#113 재범 제안).
        # 백엔드가 지원하지 않으면 무시된다.
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            cap.release()
            return None
        return cap

    def step():
        now = time.time_ns()
        if camera['cap'] is None:
            if now < camera['next_open_ns']:
                return
            camera['cap'] = open_camera()
            if camera['cap'] is None:
                camera['next_open_ns'] = now + _ns(v['reopen_period_s'])
                publish_status('OFFLINE', now, reason='OPEN_FAILED')
                log.warning(f'camera {v["device_path"]} open failed; retry in {v["reopen_period_s"]}s')
                return
        ok, frame = camera['cap'].read()
        stamp = time.time_ns()
        if not ok or frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            camera['cap'].release()
            camera['cap'] = None
            publish_status('OFFLINE', stamp, reason='READ_FAILED')
            log.warning('camera read failed; reopening')
            return
        seq['frame'] += 1
        height, width = frame.shape[:2]
        if (width, height) != (v['width'], v['height']):
            publish_status('DEGRADED', stamp, width, height, reason='SIZE_MISMATCH')
        else:
            publish_status('ONLINE', stamp, width, height)
        if slot.observe_image_size(width, height):
            log.warning(f'image size changed to {width}x{height}; watch cleared (revision {slot.revision})')
        image = frame_image(frame, stamp, v['frame_id'])
        try:
            sign = quake.update(image)
        except PerceptionError as exc:
            log.error(f'quake judge failed: {exc.detail_code}: {exc.detail}')
            return
        if sign.event is not None:
            publish_event(sign.event)
        if slot.detector is None:
            return
        try:
            result = model.predict(frame, imgsz=v['model_imgsz'], conf=v['model_conf'], verbose=False)[0]
            boxes = boxes_from_xyxyn(result.boxes.xyxyn.tolist(), result.boxes.conf.tolist())
        except Exception as exc:  # 탐지기 실패는 판정 불가로 넘긴다(소실로 세지 않음)
            log.error(f'detector failed: {exc}')
            boxes = None
        visibility = scene_visibility(sign, stream_online=True)
        watch = slot.watch
        try:
            presence = slot.detector.update(image, boxes, visibility=visibility)
        except PerceptionError as exc:
            log.error(f'watch judge failed: {exc.detail_code}: {exc.detail}')
            return
        if presence.status != 'PROCESSED':
            return
        sha256 = hashlib.sha256(image.data).hexdigest()
        publish_presence(watch, presence, stamp, sha256)
        if presence.event is not None:
            publish_event(presence.event)

    node.create_timer(v['frame_period_s'], step)
    try:
        spin_until_shutdown(rclpy, node, log)
    finally:
        if camera['cap'] is not None:
            camera['cap'].release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
