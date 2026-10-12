"""E2 synthetic preview tests; no ROS, camera, YOLO or GUI inference is claimed."""
import hashlib
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

from rescue_perception.amr_camera_support import (
    PreviewProcessor, validate_config, validate_labels, validate_rgb_compressed_format, verify_model)


def config(**overrides):
    values = dict(image_topic='camera/image_raw/compressed', image_type='compressed', annotated_topic='preview',
                  model_path='fixture.pt', model_sha256='a' * 64, model_label='survivor',
                  model_imgsz=640, model_conf=0.25, device='cpu', show=False,
                  no_frame_warn_s=5.0, log_every=10)
    return dict(values, **overrides)


class PreviewTests(unittest.TestCase):
    def test_deployment_configs_select_gpu_and_half_confidence(self):
        config_dir = Path(__file__).resolve().parents[1] / 'src/rescue_perception/config'
        for name in ('amr_camera_node.example.yaml', 'amr_target_node.example.yaml'):
            with self.subTest(config=name):
                fields = {}
                for line in (config_dir / name).read_text().splitlines():
                    key, sep, value = line.partition(':')
                    if sep and key.strip() in ('model_conf', 'device'):
                        fields[key.strip()] = value.split('#', 1)[0].strip()
                self.assertEqual(float(fields['model_conf']), 0.5)
                self.assertEqual(fields['device'], "'0'")  # ROS parameter must remain a string.

    def test_gpu_and_confidence_are_forwarded_to_detector(self):
        processor, model, bridge, _ = self.processor()
        processor = PreviewProcessor(model, bridge, config(model_conf=0.5, device='0'))
        processor.process(NS(format='jpeg', header=NS()))
        model.predict.assert_called_once_with(bridge.compressed_imgmsg_to_cv2.return_value,
                                              imgsz=640, conf=0.5, device='0', verbose=False)

    def test_required_and_invalid_parameters(self):
        for key in config():
            values = config()
            del values[key]
            with self.subTest(missing=key), self.assertRaises(ValueError):
                validate_config(values)
        for overrides in ({'image_type': 'auto'}, {'image_topic': 'preview'}, {'show': 'false'},
                          {'model_imgsz': True}, {'model_imgsz': 0}, {'model_sha256': '123'},
                          {'device': ''}, {'model_conf': float('nan')}, {'model_conf': 1.1},
                          {'no_frame_warn_s': float('inf')}, {'log_every': 0}):
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                validate_config(config(**overrides))

    def test_rgb_compressed_formats_and_depth_rejection(self):
        for fmt in ('jpeg', 'jpg', 'png', 'bgr8; jpeg compressed bgr8', 'rgb8; png compressed rgb8'):
            validate_rgb_compressed_format(fmt)
        for fmt in (None, '', '16UC1; compressedDepth png', '32FC1; compressedDepth rvl',
                    '16UC1; png compressed', 'mono16; png compressed', 'h264'):
            with self.subTest(format=fmt), self.assertRaises(ValueError):
                validate_rgb_compressed_format(fmt)
        processor, model, bridge, _ = self.processor()
        with self.assertRaisesRegex(ValueError, 'depth'):
            processor.process(NS(format='16UC1; compressedDepth png', header=NS()))
        bridge.compressed_imgmsg_to_cv2.assert_not_called()
        model.predict.assert_not_called()

    def test_hash_checked_and_missing_model_is_not_downloaded(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / 'fixture.pt'
            path.write_bytes(b'synthetic weights, not a trained model')
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            self.assertEqual(verify_model(path, digest.upper()), (str(path), digest))
            with self.assertRaisesRegex(ValueError, 'mismatch'):
                verify_model(path, '0' * 64)
            with self.assertRaises(FileNotFoundError):
                verify_model(Path(directory) / 'missing.pt', digest)

    def test_single_class_metadata(self):
        validate_labels({0: 'survivor'}, 'survivor')
        validate_labels(['survivor'], 'survivor')
        for names in ({0: 'person'}, {0: 'survivor', 1: 'person'}, {}):
            with self.assertRaises(ValueError):
                validate_labels(names, 'survivor')

    def processor(self, image_type='compressed', boxes=()):
        bridge = Mock()
        bridge.cv2_to_imgmsg.return_value = NS(header=None)
        result = Mock()
        result.boxes = boxes
        model = Mock()
        model.predict.return_value = [result]
        return PreviewProcessor(model, bridge, config(image_type=image_type)), model, bridge, result

    def test_raw_and_compressed_preserve_exact_header_and_inference_settings(self):
        for kind in ('raw', 'compressed'):
            with self.subTest(kind=kind):
                processor, model, bridge, result = self.processor(kind, boxes=[1, 2])
                message = NS(format='bgr8; jpeg compressed bgr8', header=NS(stamp=NS(sec=1700000000, nanosec=123456789), frame_id='camera_optical'))
                output, canvas, count = processor.process(message)
                self.assertEqual(output.header, message.header)
                self.assertIsNot(output.header, message.header)
                self.assertEqual(count, 2)
                decoder = bridge.imgmsg_to_cv2 if kind == 'raw' else bridge.compressed_imgmsg_to_cv2
                decoder.assert_called_once_with(message, desired_encoding='bgr8')
                model.predict.assert_called_once_with(decoder.return_value, imgsz=640, conf=0.25, device='cpu', verbose=False)
                result.plot.assert_called_once_with(labels=True, conf=True)
                self.assertIs(canvas, result.plot.return_value)
                bridge.cv2_to_imgmsg.assert_called_once_with(canvas, encoding='bgr8')
                message.header.stamp.nanosec = 1
                self.assertEqual(output.header.stamp.nanosec, 123456789)

    def test_successful_empty_detection_still_produces_annotated_frame(self):
        processor, _, _, _ = self.processor()
        self.assertEqual(processor.process(NS(format='jpeg', header=NS()))[2], 0)

    def test_decode_and_inference_failure_produce_no_output(self):
        for phase in ('decode', 'inference'):
            processor, model, bridge, _ = self.processor()
            target = bridge.compressed_imgmsg_to_cv2 if phase == 'decode' else model.predict
            target.side_effect = RuntimeError('synthetic failure')
            with self.assertRaisesRegex(RuntimeError, 'synthetic failure'):
                processor.process(NS(format='jpeg', header=NS()))
            bridge.cv2_to_imgmsg.assert_not_called()

    def test_invalid_result_is_not_successful_empty_detection(self):
        for results in ([], [NS(boxes=None)]):
            processor, model, bridge, _ = self.processor()
            model.predict.return_value = results
            with self.assertRaises(ValueError):
                processor.process(NS(format='jpeg', header=NS()))
            bridge.cv2_to_imgmsg.assert_not_called()


if __name__ == '__main__':
    unittest.main()
