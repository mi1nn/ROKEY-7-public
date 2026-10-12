"""E2 synthetic YOLO result -> PersonPipeline -> TargetLocation wire tests."""
from dataclasses import asdict, replace
from types import SimpleNamespace as NS
import math
import struct
import unittest
from uuid import UUID

from rescue_mapping import FrameAlignment, RigidTransform
from rescue_perception import AlignmentEvidence, DepthPolicy, ModelManifest, ObservationPolicy, SourceBinding, TimePolicy
from rescue_perception.person_pipeline import ConfirmationPolicy, PersonPipeline
from rescue_perception.amr_target_support import (MapStateTracker, PreparedDetector, ProjectionMetadata, check_projection_inputs,
    covariance_matrix, fill_location, fill_observation, inference_from_result, rotate_covariance, stamped_transform)

T = 1700000000123456789
FRAME = 'camera_optical'
MODEL = ModelManifest('synthetic-survivor', 'a'*64, ('survivor',))
UID = lambda n: str(UUID(int=n))


def header(stamp=T, frame=FRAME):
    return NS(stamp=NS(sec=stamp//10**9, nanosec=stamp%10**9), frame_id=frame)


def fixture(confidence=.9, depth_value=2000):
    detector = PreparedDetector()
    values = NS(xyxy=NS(tolist=lambda: [[0.,0.,4.,4.]]), conf=NS(tolist=lambda:[confidence]), cls=NS(tolist=lambda:[0.]))
    detector.prepare(T, inference_from_result(NS(boxes=values), MODEL))
    binding = SourceBinding('robot_a', 'rgbd', UID(1), 'test-clock', 'CAPTURE', 'CAPTURE', 'cal-test', 'SYNCED', 0)
    pipeline = PersonPipeline(binding=binding, alignment=AlignmentEvidence('cal-proof','cal-test',FRAME,FRAME,True),
        rectified=True, depth_policy=DepthPolicy(20000000,.2,5.,2,.5,.5), observation_policy=ObservationPolicy(2000000000),
        confirmation_policy=ConfirmationPolicy(('survivor',),.8), detector=detector)
    rgb=NS(header=header(),width=4,height=4,encoding='bgr8',is_bigendian=0,step=12,data=bytes(48))
    depth=NS(header=header(),width=4,height=4,encoding='16UC1',is_bigendian=0,step=8,data=struct.pack('<16H', *[depth_value]*16))
    info=NS(header=header(),width=4,height=4,p=[2.,0.,1.5,0.,0.,2.,1.5,0.,0.,0.,1.,0.],binning_x=0,binning_y=0,
            roi=NS(x_offset=0,y_offset=0,width=0,height=0))
    return pipeline.process(rgb,depth,info).candidates[0]


def metadata():
    a=FrameAlignment('robot_a','map','mission',UID(2),0,1,'mapping-owner','test-clock',
                     RigidTransform((.5,-.5,0.),(0.,0.,0.,1.)),T-1000000000,T+10000000000,.1,0.,'VALID')
    return ProjectionMetadata(a,True,'test-covariance',(.01,.04,.09))


def map_state(**overrides):
    values=dict(robot_id='robot_a',current_native_map_frame='map',map_session_id=UID(2),map_revision=0,
                generated_at_ns=T-100,correction_detection_status='KNOWN',localization_quality='VALID')
    return NS(**dict(values,**overrides))


def wire_observation():
    return NS(time=NS(), sensor_position=NS(), evidence=NS())


class TargetTests(unittest.TestCase):
    def check(self,o=None,m=None,state=None,now=T+100):
        check_projection_inputs(o or fixture().observation,m or metadata(),state or map_state(),now_ns=now,
            max_map_age_ns=1000000000,receiver_clock_error_ns=0,time_policy=TimePolicy(1000000000,0,10000000,0))

    def test_result_conversion_and_exact_stamp_binding(self):
        detector=PreparedDetector()
        result=NS(boxes=NS(xyxy=NS(tolist=lambda:[]),conf=NS(tolist=lambda:[]),cls=NS(tolist=lambda:[])))
        inference=inference_from_result(result,MODEL)
        self.assertEqual(inference.outputs,())
        detector.prepare(T,inference)
        self.assertIs(detector.infer(None,T),inference)
        self.assertIsNone(detector.infer(None,T+1))
        with self.assertRaises(ValueError): inference_from_result(NS(boxes=None),MODEL)
        result.boxes.cls=NS(tolist=lambda:[2.])
        with self.assertRaises(ValueError): inference_from_result(result,MODEL)

    def test_rgbd_point_and_mission_message_preserve_provenance(self):
        candidate=fixture()
        self.assertEqual(candidate.status,'CONFIRMED')
        o=candidate.observation
        self.assertEqual(o.localization.point_optical,(0.,0.,2.))
        self.check(o)
        covariance=covariance_matrix((.01,.04,.09))
        observed=fill_observation(wire_observation(),o,covariance)
        message=NS(frame_ref=NS(),mission_position=NS())
        result=fill_location(message,observed,o,metadata(),RigidTransform((1.,2.,0.),(0.,0.,0.,1.)),covariance)
        self.assertEqual((result.mission_position.x,result.mission_position.y,result.mission_position.z),(1.5,1.5,2.))
        self.assertEqual(result.observation.time.observed_at_ns,T)
        self.assertEqual(result.observation.time.valid_until_ns,T+2000000000)
        self.assertEqual(result.frame_ref.map_session_id,UID(2))
        self.assertEqual(result.frame_ref.mission_frame,'mission')
        self.assertEqual(result.projection_status,'VALID')
        self.assertAlmostEqual(result.covariance[0],.02)
        self.assertEqual(fill_location(NS(frame_ref=NS(),mission_position=NS()),observed,o,metadata(),
            RigidTransform((1.,2.,0.),(0.,0.,0.,1.)),covariance).location_id,result.location_id)

    def test_invalid_depth_is_unknown_not_origin_or_confirmed(self):
        candidate=fixture(depth_value=0)
        self.assertEqual(candidate.status,'UNCONFIRMED')
        message=fill_observation(wire_observation(),candidate.observation,covariance_matrix((1.,1.,1.)))
        self.assertEqual(message.localization_status,'UNKNOWN')
        self.assertTrue(math.isnan(message.sensor_position.x))
        with self.assertRaises(Exception): self.check(candidate.observation)
        self.assertEqual(fixture(confidence=.7).status,'UNCONFIRMED')

    def test_unknown_clock_stale_map_revision_and_expired_alignment_are_rejected(self):
        for overrides in ({'map_revision':1},{'map_session_id':UID(3)},{'robot_id':'robot_b'},
                          {'correction_detection_status':'UNKNOWN'},{'localization_quality':'UNKNOWN'},
                          {'generated_at_ns':T-2000000000},{'generated_at_ns':T+10}):
            with self.subTest(overrides=overrides), self.assertRaises(Exception): self.check(state=map_state(**overrides))
        with self.assertRaises(Exception): self.check(replace(fixture().observation,clock_state='UNKNOWN',clock_error_bound_ns=None))
        with self.assertRaises(Exception): self.check(m=replace(metadata(),alignment=replace(metadata().alignment,valid_until_ns=T+10)))
        with self.assertRaises(Exception): self.check(now=T+3000000000)
        with self.assertRaises(Exception): self.check(m=replace(metadata(),covariance_verified=False))

    def test_mapping_metadata_requires_explicit_covariance_and_valid_status(self):
        values=asdict(metadata())
        restored=ProjectionMetadata.from_dict(values)
        self.assertEqual(restored,metadata())
        for override in ({'covariance_verified':False},{'covariance_evidence_id':''}):
            with self.assertRaises(ValueError): ProjectionMetadata.from_dict(dict(values,**override))
        values['alignment'].pop('status')
        with self.assertRaises(ValueError): ProjectionMetadata.from_dict(values)
        with self.assertRaises(ValueError):
            check_projection_inputs(fixture().observation,metadata(),None,now_ns=T+100,
                max_map_age_ns=1000000000,receiver_clock_error_ns=0,time_policy=TimePolicy(1000000000,0,0,0))

    def test_map_revision_or_retired_session_replay_cannot_restore_permission(self):
        tracker=MapStateTracker('robot_a')
        tracker.update(map_state())
        tracker.update(map_state(map_revision=1,generated_at_ns=T))
        with self.assertRaises(ValueError): tracker.update(map_state(map_revision=0,generated_at_ns=T+1))
        self.assertEqual(tracker.current.map_revision,1)
        tracker.update(map_state(map_session_id=UID(3),generated_at_ns=T+2))
        with self.assertRaises(ValueError): tracker.update(map_state(generated_at_ns=T+3))
        self.assertEqual(tracker.current.map_session_id,UID(3))

    def test_tf_latest_fallback_and_wrong_frame_are_rejected(self):
        transform=NS(header=header(frame='map'),child_frame_id=FRAME,
                     transform=NS(translation=NS(x=1.,y=2.,z=0.),rotation=NS(x=0.,y=0.,z=0.,w=1.)))
        rigid=stamped_transform(transform,source_frame=FRAME,target_frame='map',observed_at_ns=T)
        self.assertEqual(rigid.apply((0.,0.,2.)),(1.,2.,2.))
        transform.header=header(T+1,'map')
        with self.assertRaises(ValueError): stamped_transform(transform,source_frame=FRAME,target_frame='map',observed_at_ns=T)
        transform.header=header(frame='odom')
        with self.assertRaises(ValueError): stamped_transform(transform,source_frame=FRAME,target_frame='map',observed_at_ns=T)

    def test_covariance_rotates_with_coordinate_frame(self):
        covariance=rotate_covariance(covariance_matrix((1.,4.,9.)),RigidTransform.planar(0.,0.,math.pi/2))
        self.assertAlmostEqual(covariance[0],4.)
        self.assertAlmostEqual(covariance[4],1.)
        self.assertAlmostEqual(covariance[8],9.)
        for values in ((0.,1.,1.),(-1.,1.,1.),(math.nan,1.,1.)):
            with self.assertRaises(ValueError): covariance_matrix(values)


if __name__=='__main__': unittest.main()
