"""Webcam judge video (CompressedImage jpeg) -> HTTP multipart MJPEG for the console (2026-10-13).

Standard library http.server only, so no web_video_server install. The console shows it with
`--camera cam_fixed=http://<host>:<port>/<path>`. Each frame is sent once. When no new frame
arrives within stale_s the stream is closed; the last frame is never repeated, so a stopped
camera does not look live. A request while there is no fresh frame gets 503.
"""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import sys
import threading
import time

BOUNDARY = 'frame'
PARAMS = {  # name -> rcl_interfaces ParameterType name; every one is required, none has a default
    'image_topic': 'STRING',   # the webcam_node debug_image_topic (CompressedImage, jpeg)
    'bind_host': 'STRING', 'port': 'INTEGER', 'path': 'STRING',
    'stale_s': 'DOUBLE',       # no new frame for this long -> streams close, new requests get 503
}


def is_jpeg_format(fmt):
    words = fmt.lower().replace(';', ' ').split()
    return ('jpeg' in words or 'jpg' in words) and not any('depth' in w or w == 'png' for w in words)


class FrameHub:
    """The latest jpeg with a sequence number; waiting readers wake on each new frame."""

    def __init__(self, stale_s, clock=time.monotonic):
        if type(stale_s) is not float or not math.isfinite(stale_s) or stale_s <= 0:
            raise ValueError('stale_s must be finite and positive')
        self.stale_s, self._clock = stale_s, clock
        self._cond = threading.Condition()
        self._seq, self._jpeg, self._at, self._closed = 0, None, None, False

    def put(self, jpeg):
        with self._cond:
            self._seq += 1
            self._jpeg, self._at = bytes(jpeg), self._clock()
            self._cond.notify_all()

    def fresh(self):
        """(seq, jpeg) of the latest frame if it is newer than stale_s, else None."""
        with self._cond:
            if self._closed or self._jpeg is None or self._clock() - self._at >= self.stale_s:
                return None
            return self._seq, self._jpeg

    def next_after(self, seq):
        """The first frame after ``seq`` within stale_s, or None (camera stopped or hub closed)."""
        with self._cond:
            self._cond.wait_for(lambda: self._closed or self._seq > seq, timeout=self.stale_s)
            if (self._closed or self._seq <= seq
                    or self._clock() - self._at >= self.stale_s):
                return None
            return self._seq, self._jpeg

    def close(self):
        with self._cond:
            self._closed = True
            self._cond.notify_all()


def make_server(hub, host, port, path, log=None):
    """ThreadingHTTPServer serving ``path`` as MJPEG from ``hub``; every other path is 404."""

    class Handler(BaseHTTPRequestHandler):
        # Socket timeout for every read and write: a client that stops reading cannot hold this
        # thread longer than one stale period (#153 left the write blocked for good).
        timeout = hub.stale_s

        def log_message(self, fmt, *args):
            if log is not None:
                log(f'{self.client_address[0]} {fmt % args}')

        def do_GET(self):
            if self.path.split('?', 1)[0] != path:
                self.send_error(404)
                return
            frame = hub.fresh()
            if frame is None:
                self.send_error(503, 'no fresh webcam frame')
                return
            try:
                self.send_response(200)
                self.send_header('Content-Type', f'multipart/x-mixed-replace; boundary={BOUNDARY}')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Connection', 'close')
                self.end_headers()
                while frame is not None:
                    seq, jpeg = frame
                    self.wfile.write(f'--{BOUNDARY}\r\nContent-Type: image/jpeg\r\n'
                                     f'Content-Length: {len(jpeg)}\r\n\r\n'.encode() + jpeg + b'\r\n')
                    self.wfile.flush()
                    frame = hub.next_after(seq)
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                pass
            self.close_connection = True

    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    return server


def main(args=None):
    import rclpy
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.exceptions import ParameterUninitializedException
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CompressedImage

    from .camera_utils import git_sha

    rclpy.init(args=args)
    node = Node('mjpeg_relay')
    log = node.get_logger()
    server = hub = None
    serving = False
    try:
        values = {}
        for name, kind in PARAMS.items():
            node.declare_parameter(name, Parameter.Type[kind], ParameterDescriptor(description='required; no default'))
            try:
                values[name] = node.get_parameter(name).value
            except ParameterUninitializedException:
                values[name] = None
        missing = sorted(name for name, value in values.items() if value is None or value == '')
        if missing:
            log.fatal(f'missing parameters: {missing}')
            return 2
        try:
            hub = FrameHub(values['stale_s'])
            if not values['path'].startswith('/') or not 0 < values['port'] < 65536:
                raise ValueError('path must start with / and port must be 1..65535')
            server = make_server(hub, values['bind_host'], values['port'], values['path'],
                                 log=lambda text: log.info(text))
        except (ValueError, OSError) as exc:
            log.fatal(f'invalid parameters: {exc}')
            return 2
        warned = set()

        def on_image(msg):
            if not is_jpeg_format(msg.format):
                if msg.format not in warned:
                    warned.add(msg.format)
                    log.warning(f'frame format {msg.format!r} is not jpeg; not relayed')
                return
            hub.put(msg.data)

        # Same QoS as the webcam_node debug publisher (BEST_EFFORT, depth 1).
        node.create_subscription(CompressedImage, values['image_topic'], on_image,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        threading.Thread(target=server.serve_forever, name='mjpeg-http', daemon=True).start()
        serving = True
        log.info(f'mjpeg_relay git={git_sha()} parameters={json.dumps(values, sort_keys=True)} '
                 f'url=http://{values["bind_host"]}:{server.server_address[1]}{values["path"]}')
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if hub is not None:
            hub.close()
        if server is not None:
            if serving:  # shutdown() waits for serve_forever, so only after it started
                server.shutdown()
            server.server_close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
