"""E2 synthetic camera-to-base TF; no physical robot motion."""
import unittest
from types import SimpleNamespace as S
from rescue_perception.target_utils import fill_base_point, OneShotTarget

def observation():
    return S(observed_at_ns=1_000_000_123,frame=S(frame_id='optical',width=30,height=30),
             localization=S(point_optical=(-.4,.3,1.35)),observation_id='test',
             detection=S(bbox=(3.,3.,27.,27.)))

def transform(stamp=0,parent='base_link',child='optical'):
    sec,nsec=divmod(stamp,10**9)
    return S(header=S(frame_id=parent,stamp=S(sec=sec,nanosec=nsec)),child_frame_id=child,
             transform=S(translation=S(x=.1,y=0.,z=.2),rotation=S(x=-.5,y=.5,z=-.5,w=.5)))

def message():
    return S(header=S(frame_id='',stamp=S(sec=0,nanosec=0)),point=S(x=0.,y=0.,z=0.))

class BasePointTests(unittest.TestCase):
    def test_capture_time_lookup_and_optical_to_base_axis_mapping(self):
        calls=[]
        def lookup(*args):calls.append(args);return transform()
        o=observation();p=fill_base_point(message(),o,lookup,base_frame='base_link')
        self.assertEqual(calls,[('base_link','optical',o.observed_at_ns)])
        self.assertEqual(p.header.frame_id,'base_link')
        self.assertEqual((p.header.stamp.sec,p.header.stamp.nanosec),(1,123))
        for actual,expected in zip((p.point.x,p.point.y,p.point.z),(1.45,.4,-.1)):
            self.assertAlmostEqual(actual,expected)
    def test_wrong_frames_and_non_capture_dynamic_tf_are_refused(self):
        o=observation()
        for tf in (transform(parent='map'),transform(child='other'),transform(stamp=o.observed_at_ns+1)):
            with self.assertRaises(ValueError):
                fill_base_point(message(),o,lambda *args:tf,base_frame='base_link')
    def test_dynamic_transform_at_capture_time_is_accepted(self):
        o=observation()
        self.assertEqual(fill_base_point(message(),o,lambda *args:transform(stamp=o.observed_at_ns),
                                        base_frame='base_link').header.stamp.nanosec,123)
    def test_tf_unavailable_does_not_latch_sent_and_next_frame_retries(self):
        o=observation();candidate=S(status='CONFIRMED',observation=o)
        sender=OneShotTarget()
        def unavailable(*args):raise RuntimeError('no capture-time TF')
        with self.assertRaises(RuntimeError):
            sender.send(candidate,lambda obj:fill_base_point(message(),obj,unavailable,base_frame='base_link'))
        self.assertEqual(sender.state,'SEARCHING')
        self.assertTrue(sender.send(candidate,lambda obj:fill_base_point(message(),obj,lambda *args:transform(),base_frame='base_link')))
        self.assertEqual(sender.state,'SENT')
