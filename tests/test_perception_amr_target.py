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
from rescue_perception.target_utils import (PreparedDetector, covariance_matrix, fill_observation, inference_from_result)
from rescue_perception.map_projection import (MapStateTracker, ProjectionMetadata, check_projection_inputs,
    fill_location, rotate_covariance, stamped_transform)

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


class BasePointTests(unittest.TestCase):
    def transform(self, *, stamp=T, source=FRAME, target='base_link'):
        # optical right/down/forward -> base forward/left/up, with camera offset.
        return NS(header=header(stamp,target),child_frame_id=source,
                  transform=NS(translation=NS(x=.2,y=0.,z=.3),
                               rotation=NS(x=-.5,y=.5,z=-.5,w=.5)))

    def test_base_point_rotates_translates_and_preserves_capture_stamp(self):
        from unittest.mock import Mock
        from rescue_perception.target_utils import fill_base_point
        lookup=Mock(return_value=self.transform())
        o=fixture().observation
        point=fill_base_point(NS(header=header(),point=NS()),o,lookup,base_frame='base_link')
        lookup.assert_called_once_with('base_link',FRAME,T)
        self.assertEqual(point.header.frame_id,'base_link')
        self.assertEqual((point.header.stamp.sec,point.header.stamp.nanosec),(T//10**9,T%10**9))
        self.assertAlmostEqual(point.point.x,2.2)
        self.assertAlmostEqual(point.point.y,0.)
        self.assertAlmostEqual(point.point.z,.3)
        self.assertEqual(o.frame.frame_id,FRAME)
        self.assertEqual(o.localization.point_optical,(0.,0.,2.))

    def test_static_mount_transform_is_valid_after_capture_time_lookup(self):
        from rescue_perception.target_utils import fill_base_point
        point=fill_base_point(NS(header=header(),point=NS()),fixture().observation,
                              lambda *args:self.transform(stamp=0),base_frame='base_link')
        self.assertAlmostEqual(point.point.x,2.2)
        self.assertEqual(point.header.stamp.nanosec,T%10**9)

    def test_missing_tf_unknown_depth_wrong_frames_and_wrong_time_produce_no_point(self):
        from unittest.mock import Mock
        from rescue_perception.target_utils import fill_base_point
        for transform in (self.transform(source='wrong_camera'),self.transform(target='odom'),self.transform(stamp=T+1)):
            with self.subTest(transform=transform),self.assertRaises(ValueError):
                fill_base_point(NS(header=header(),point=NS()),fixture().observation,lambda *args:transform,base_frame='base_link')
        lookup=Mock(side_effect=RuntimeError('TF unavailable'))
        with self.assertRaises(RuntimeError):
            fill_base_point(NS(header=header(),point=NS()),fixture().observation,lookup,base_frame='base_link')
        lookup.reset_mock()
        with self.assertRaises(ValueError):
            fill_base_point(NS(header=header(),point=NS()),fixture(depth_value=0).observation,lookup,base_frame='base_link')
        lookup.assert_not_called()


class OneShotTests(unittest.TestCase):
    def candidate(self, *, confidence=.9, depth=2000, bbox=(.5,.5,3.5,3.5)):
        c=fixture(confidence=confidence,depth_value=depth)
        return replace(c,observation=replace(c.observation,
            detection=replace(c.observation.detection,bbox=bbox)))

    def test_first_confirmed_interior_target_is_sent_once_for_entire_run(self):
        from unittest.mock import Mock
        from rescue_perception.target_utils import OneShotTarget
        sender=OneShotTarget();publish=Mock();candidate=self.candidate(confidence=.8)
        self.assertEqual(sender.state,'SEARCHING')
        self.assertTrue(sender.send(candidate,publish))
        self.assertEqual(sender.state,'SENT')
        self.assertEqual(sender.observation_id,candidate.observation.observation_id)
        self.assertFalse(sender.send(candidate,publish))
        self.assertFalse(sender.send(self.candidate(),publish))
        publish.assert_called_once_with(candidate.observation)
        self.assertEqual(OneShotTarget().state,'SEARCHING')

    def test_low_confidence_invalid_depth_and_all_four_image_edges_wait(self):
        from unittest.mock import Mock
        from rescue_perception.target_utils import OneShotTarget
        sender=OneShotTarget();publish=Mock()
        for candidate in (self.candidate(confidence=.79),self.candidate(depth=0),
                          *(self.candidate(bbox=b) for b in ((0.,.5,3.5,3.5),(.5,0.,3.5,3.5),
                                                            (.5,.5,4.,3.5),(.5,.5,3.5,4.)))):
            with self.subTest(candidate=candidate):
                self.assertFalse(sender.send(candidate,publish))
                self.assertEqual(sender.state,'SEARCHING')
        publish.assert_not_called()
        self.assertTrue(sender.send(self.candidate(),publish))
        publish.assert_called_once()

    def test_publisher_failure_does_not_latch_sent(self):
        from unittest.mock import Mock
        from rescue_perception.target_utils import OneShotTarget
        sender=OneShotTarget();publish=Mock(side_effect=RuntimeError('publish failed'))
        with self.assertRaises(RuntimeError):sender.send(self.candidate(),publish)
        self.assertEqual(sender.state,'SEARCHING')
        self.assertIsNone(sender.observation_id)
        publish.side_effect=None
        self.assertTrue(sender.send(self.candidate(),publish))


if __name__=='__main__': unittest.main()
