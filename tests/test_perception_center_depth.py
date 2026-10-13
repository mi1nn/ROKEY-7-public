import unittest
from array import array
from dataclasses import replace
from types import SimpleNamespace as S
from rescue_perception.adapter import SourceBinding, frame_meta, locate_center_depth
from rescue_perception.core import AlignmentEvidence,CameraIntrinsics,DepthPolicy,Detection

class CenterDepthTests(unittest.TestCase):
    def setUp(self):
        self.binding=SourceBinding('robot_a','amr','00000000-0000-4000-8000-000000000001','clock','CAPTURE','CAPTURE','cal','UNKNOWN',None)
        self.rgb=S(header=S(frame_id='optical',stamp=S(sec=1,nanosec=0)),width=30,height=30)
        self.depth=S(header=self.rgb.header,width=30,height=30,encoding='16UC1',is_bigendian=0,step=60,data=array('H',[3000])*900)
        self.intr=CameraIntrinsics('optical','cal',30,30,20.,20.,15.,15.,True)
        self.align=AlignmentEvidence('verified','cal','optical','optical',True)
        self.policy=DepthPolicy(20_000_000,.2,5.,10,.5,.5)
        self.det=Detection(0,'survivor',.97,(3.,3.,27.,27.),None,'DETECTION_ONLY')
    def locate(self,**kw):
        return locate_center_depth(frame_meta(self.rgb,self.binding,'CAPTURE','rgb'),self.depth,self.binding,self.intr,self.align,self.det,self.policy,radius=kw.get('radius',5))
    def fill(self,x,y,r,value):
        for v in range(y-r,y+r+1):
            for u in range(x-r,x+r+1):self.depth.data[v*30+u]=value
    def test_background_outside_center_does_not_contaminate_depth(self):
        self.fill(14,14,5,1500)
        result=self.locate();self.assertEqual(result.reason,'LOCATED');self.assertAlmostEqual(result.point_optical[2],1.5)
        self.assertEqual(result.valid_pixels,121)
    def test_raw_pixel_is_mapped_before_depth_lookup_and_xyz(self):
        self.det=replace(self.det,bbox=(1.,6.,9.,14.))
        self.intr=replace(self.intr,rectified=False,undistort=lambda ps:[(u+10,v) for u,v in ps],undistortion_id='synthetic-shift')
        self.fill(14,10,2,1400)
        result=self.locate(radius=2)
        self.assertEqual(result.reason,'LOCATED');self.assertAlmostEqual(result.point_optical[2],1.4)
        self.assertAlmostEqual(result.point_optical[0],(14.5-15.)*1.4/20.)
        self.assertEqual(result.undistortion_id,'synthetic-shift')
    def test_scale_is_applied_exactly_once(self):
        self.fill(14,14,5,1500);self.policy=replace(self.policy,scale=.9,offset_m=.1)
        self.assertAlmostEqual(self.locate().point_optical[2],1.45)
    def test_invalid_center_does_not_use_background(self):
        self.fill(14,14,5,0);self.assertEqual(self.locate().reason,'DEPTH_INSUFFICIENT')
    def test_original_dimensions_still_must_match(self):
        self.depth.width=29;self.assertEqual(self.locate().reason,'DIMENSION_MISMATCH')
    def test_alignment_and_time_gates_remain(self):
        self.align=replace(self.align,verified=False);self.assertEqual(self.locate().reason,'ALIGNMENT_UNVERIFIED')
        self.align=replace(self.align,verified=True);self.depth.header=S(frame_id='optical',stamp=S(sec=1,nanosec=30_000_000))
        self.assertEqual(self.locate().reason,'UNSYNCHRONIZED')
    def test_big_endian_depth_with_row_padding(self):
        import struct
        self.fill(14,14,5,1500)
        self.depth.data=b''.join(struct.pack('>30H',*self.depth.data[y*30:(y+1)*30])+b'pad!' for y in range(30))
        self.depth.is_bigendian=1;self.depth.step=64
        self.assertAlmostEqual(self.locate().point_optical[2],1.5)
    def test_roi_is_clipped_to_object_and_image(self):
        self.det=replace(self.det,bbox=(1.,1.,5.,5.));self.depth.data=array('H',[1500])*900
        result=self.locate();self.assertEqual(result.reason,'LOCATED');self.assertEqual(result.valid_pixels,16)
