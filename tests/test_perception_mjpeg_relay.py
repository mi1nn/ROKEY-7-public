"""Webcam judge video -> HTTP multipart MJPEG for the console (2026-10-13). Real HTTP on localhost, no ROS.

The console shows `--camera cam_fixed=http://<host>:<port>/<path>` as an <img>. A frame is sent
once; when the camera stops, the stream closes instead of repeating the last frame.
"""
import http.client
import threading
import time
import unittest

from rescue_perception.mjpeg_relay import FrameHub, is_jpeg_format, make_server

STALE_S = 0.3


class RelayFixture(unittest.TestCase):
    def setUp(self):
        self.hub = FrameHub(STALE_S)
        self.server = make_server(self.hub, 'localhost', 0, '/stream')
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.addCleanup(self.server.server_close)
        self.addCleanup(self.server.shutdown)
        self.addCleanup(self.hub.close)

    def get(self, path='/stream'):
        connection = http.client.HTTPConnection('localhost', self.server.server_address[1], timeout=5)
        self.addCleanup(connection.close)
        connection.request('GET', path)
        return connection.getresponse()


def read_part(response):
    """One multipart part -> jpeg bytes, or None when the stream ended."""
    line = response.fp.readline()
    while line in (b'\r\n',):
        line = response.fp.readline()
    if not line:
        return None
    assert line == b'--frame\r\n', line
    headers = {}
    while True:
        line = response.fp.readline()
        if line == b'\r\n':
            break
        key, _, value = line.decode().partition(':')
        headers[key.strip().lower()] = value.strip()
    assert headers['content-type'] == 'image/jpeg'
    data = response.fp.read(int(headers['content-length']))
    response.fp.readline()
    return data


class MjpegRelayTests(RelayFixture):
    def test_no_frame_yet_is_503(self):
        self.assertEqual(503, self.get().status)

    def test_unknown_path_is_404(self):
        self.hub.put(b'\xff\xd8a\xff\xd9')
        self.assertEqual(404, self.get('/other').status)

    def test_frames_stream_once_each_then_the_stream_closes_when_the_camera_stops(self):
        self.hub.put(b'A')
        response = self.get()
        self.assertEqual(200, response.status)
        self.assertEqual('multipart/x-mixed-replace; boundary=frame', response.getheader('Content-Type'))
        self.assertEqual('no-store', response.getheader('Cache-Control'))
        self.assertEqual(b'A', read_part(response))
        self.hub.put(b'B')
        self.assertEqual(b'B', read_part(response))
        started = time.monotonic()
        self.assertIsNone(read_part(response))  # no new frame -> closed, B is not repeated
        self.assertLess(time.monotonic() - started, STALE_S + 2)

    def test_old_last_frame_is_not_served(self):
        self.hub.put(b'A')
        time.sleep(STALE_S + 0.1)
        self.assertEqual(503, self.get().status)

    def test_closing_the_hub_ends_open_streams(self):
        self.hub.put(b'A')
        response = self.get()
        self.assertEqual(b'A', read_part(response))
        self.hub.close()
        self.assertIsNone(read_part(response))


class FormatTests(unittest.TestCase):
    def test_next_after_does_not_send_a_frame_that_expired_during_a_slow_write(self):
        now = [0.0]
        hub = FrameHub(1.0, clock=lambda: now[0])
        hub.put(b'first')
        sequence, _ = hub.fresh()
        hub.put(b'last')
        now[0] = 2.0
        self.assertIsNone(hub.fresh())
        self.assertIsNone(hub.next_after(sequence))

    def test_only_jpeg_is_relayed(self):
        self.assertTrue(is_jpeg_format('jpeg'))
        self.assertTrue(is_jpeg_format('bgr8; jpeg compressed bgr8'))
        for fmt in ('png', 'bgr8; png compressed bgr8', '16UC1; compressedDepth', ''):
            self.assertFalse(is_jpeg_format(fmt), fmt)

    def test_stale_must_be_positive(self):
        for bad in (0.0, -1.0, float('nan')):
            with self.assertRaises(ValueError):
                FrameHub(bad)


if __name__ == '__main__':
    unittest.main()
