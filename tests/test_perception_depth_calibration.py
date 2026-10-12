"""Robot6 PR #126 depth correction: synthetic E2, no physical sensor claims."""
import json
from pathlib import Path
from importlib.resources import files
from tempfile import TemporaryDirectory
import unittest

from rescue_perception import DepthPolicy, PerceptionError
from test_perception_amr_target import fixture, metadata, wire_observation
from rescue_perception.target_utils import covariance_matrix, fill_observation
from rescue_perception.map_projection import fill_location

SCALE = 0.82 / ((0.854 + 0.853 + 0.857) / 3)


class DepthCalibrationTests(unittest.TestCase):
    def policy(self, **changes):
        return DepthPolicy(20000000, .2, 5., 2, .5, .5, **changes)

    def candidate(self, encoding, value, *, scale=SCALE, offset=0.):
        from test_perception_amr_target import FRAME, MODEL, T, UID, header
        from types import SimpleNamespace as NS
        import struct
        from rescue_perception import AlignmentEvidence, ObservationPolicy, SourceBinding
        from rescue_perception.person_pipeline import ConfirmationPolicy, PersonPipeline
        from rescue_perception.target_utils import PreparedDetector, inference_from_result
        detector = PreparedDetector()
        boxes = NS(xyxy=NS(tolist=lambda:[[2., 0., 4., 4.]]), conf=NS(tolist=lambda:[.9]), cls=NS(tolist=lambda:[0.]))
        detector.prepare(T, inference_from_result(NS(boxes=boxes), MODEL))
        pipeline = PersonPipeline(binding=SourceBinding('robot_a','rgbd',UID(1),'test-clock','CAPTURE','CAPTURE','cal-test','SYNCED',0),
            alignment=AlignmentEvidence('proof','cal-test',FRAME,FRAME,True), rectified=True,
            depth_policy=self.policy(scale=scale, offset_m=offset), observation_policy=ObservationPolicy(2000000000),
            confirmation_policy=ConfirmationPolicy(('survivor',),.8), detector=detector)
        rgb = NS(header=header(),width=4,height=4,encoding='bgr8',step=12,data=bytes(48))
        code, step = ('H',8) if encoding=='16UC1' else ('f',16)
        depth = NS(header=header(),width=4,height=4,encoding=encoding,is_bigendian=0,step=step,
                   data=struct.pack('<16'+code,*[value]*16))
        info = NS(header=header(),width=4,height=4,p=[2.,0.,1.5,0.,0.,2.,1.5,0.,0.,0.,1.,0.],
                  binning_x=0,binning_y=0,roi=NS(x_offset=0,y_offset=0,width=0,height=0))
        return pipeline.process(rgb,depth,info).candidates[0]

    def test_pr126_scale_corrects_xyz_once_after_mm_or_m_conversion(self):
        for encoding, value in (('16UC1',857),('32FC1',.857)):
            with self.subTest(encoding=encoding):
                candidate=self.candidate(encoding,value)
                self.assertEqual(candidate.status,'CONFIRMED')
                x,y,z=candidate.observation.localization.point_optical
                self.assertAlmostEqual(z,.857*SCALE,places=6)
                self.assertAlmostEqual(x,z/2,places=6)
                self.assertAlmostEqual(y,0.,places=6)

    def test_invalid_raw_depth_is_never_resurrected_by_positive_offset(self):
        for encoding, value in (('16UC1',0),('32FC1',0.),('32FC1',-1.),('32FC1',float('nan')),('32FC1',float('inf'))):
            with self.subTest(encoding=encoding,value=value):
                candidate=self.candidate(encoding,value,offset=1.)
                self.assertEqual(candidate.status,'UNCONFIRMED')
                self.assertIsNone(candidate.observation.localization.point_optical)

    def test_offset_and_corrected_depth_limits(self):
        self.assertAlmostEqual(self.candidate('16UC1',857,offset=.01).observation.localization.point_optical[2],.857*SCALE+.01)
        self.assertIsNone(self.candidate('16UC1',857,offset=-1.).observation.localization.point_optical)

    def test_invalid_scale_or_offset_rejected(self):
        for changes in ({'scale':0.},{'scale':-1.},{'scale':float('nan')},{'scale':float('inf')},{'scale':True},
                        {'offset_m':float('nan')},{'offset_m':float('inf')},{'offset_m':False}):
            with self.subTest(changes=changes), self.assertRaises(PerceptionError): self.policy(**changes)

    def test_identity_policy_preserves_existing_behavior(self):
        self.assertEqual(self.policy().scale,1.)
        self.assertEqual(self.policy().offset_m,0.)
        self.assertEqual(fixture().observation.localization.point_optical,(0.,0.,2.))

    def test_corrected_point_flows_into_target_location(self):
        from rescue_mapping import RigidTransform
        from types import SimpleNamespace as NS
        o=self.candidate('16UC1',857).observation
        covariance=covariance_matrix((.01,.04,.09))
        wire=fill_observation(wire_observation(),o,covariance)
        location=fill_location(NS(frame_ref=NS(),mission_position=NS()),wire,o,metadata(),RigidTransform.identity(),covariance)
        self.assertAlmostEqual(location.mission_position.z,.857*SCALE)
        self.assertEqual(wire.time.observed_at_ns,o.observed_at_ns)

    def test_robot6_profile_and_physical_robot_guard(self):
        from rescue_perception.target_utils import load_depth_calibration
        path='package:robot6_depth.json'  # Same single profile in source and installed package.
        calibration=load_depth_calibration(path,physical_robot_id='robot6')
        self.assertAlmostEqual(calibration.scale,SCALE)
        self.assertEqual(calibration.offset_m,0.)
        self.assertEqual(len(calibration.sha256),64)
        with self.assertRaisesRegex(ValueError,'robot'): load_depth_calibration(path,physical_robot_id='robot5')
        for bad in (float('nan'),0.,-1.):
            with TemporaryDirectory() as d:
                data=json.loads(files('rescue_perception').joinpath('config').joinpath('robot6_depth.json').read_text()); data['scale']=bad
                other=Path(d)/'bad.json'; other.write_text(json.dumps(data))
                with self.assertRaises(ValueError): load_depth_calibration(other,physical_robot_id='robot6')

    def test_runtime_only_profile_loads_without_provenance(self):
        from rescue_perception.target_utils import load_depth_calibration
        runtime=dict(calibration_id='robot6_depth',physical_robot_id='robot6',
                     input_unit='m',input_state='raw_uncorrected',scale=SCALE,offset_m=0.)
        with TemporaryDirectory() as directory:
            path=Path(directory)/'depth.json';path.write_text(json.dumps(runtime))
            calibration=load_depth_calibration(path,physical_robot_id='robot6')
            self.assertAlmostEqual(calibration.scale,SCALE)
            self.assertEqual(calibration.calibration_id,'robot6_depth')
        packaged=json.loads(files('rescue_perception').joinpath('config/robot6_depth.json').read_text())
        self.assertEqual(set(packaged),set(runtime))
        self.assertEqual(packaged['calibration_id'],'robot6_depth')

    def test_node_configuration_connects_profile_to_depth_policy(self):
        from rescue_perception.target_utils import target_config
        from rescue_perception.target_utils import SETTINGS
        values=dict(SETTINGS)
        self.assertEqual(values['physical_robot_id'],'robot6')
        self.assertEqual(values['depth_calibration_path'],'package:robot6_depth.json')
        binding,alignment,depth,*_=target_config(values,fixture().observation.source_boot_id)
        self.assertAlmostEqual(depth.scale,SCALE)
        self.assertIn('robot6_depth',binding.calibration_id)
        self.assertEqual(binding.calibration_id,alignment.calibration_id)


if __name__=='__main__': unittest.main()
