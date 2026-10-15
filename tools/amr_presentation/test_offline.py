#!/usr/bin/env python3
"""E2 tests: no ROS, model, GUI, robot or network required."""
import hashlib
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace as NS
import unittest
from dataclasses import replace

import numpy as np
import yolo_depth_viewer as viewer
from amr_source.core import FrameMeta
from roi_comparison import Collector, evaluate
from roi_report import summarize


class OfflineTests(unittest.TestCase):
    def setUp(self):
        self.binding=viewer.SourceBinding('robot_a','test','12345678-1234-1234-1234-123456789abc',
            'test','CAPTURE','CAPTURE','cal','UNKNOWN',None)
        self.alignment=viewer.AlignmentEvidence('test','cal','optical','optical',True)
        self.policy=viewer.DepthPolicy(20_000_000,.2,5.,10,.5,.5)
        header=NS(frame_id='optical',stamp=NS(sec=10,nanosec=0))
        self.image=np.full((480,640),2000,np.uint16)
        self.dm=NS(header=header,width=640,height=480,encoding='16UC1',is_bigendian=False,step=1280,data=self.image.tobytes())
        self.ci=NS(header=header,width=640,height=480,binning_x=0,binning_y=0,
            roi=NS(x_offset=0,y_offset=0,width=0,height=0),p=[500.,0.,320.,0.,0.,500.,240.,0.,0.,0.,1.,0.])
        self.meta=FrameMeta('optical',10_000_000_000,'test','CAPTURE',640,480,'cal')
        self.selected=viewer.DetectorOutput('survivor',.97,(100.,100.,200.,300.),None)

    def rows(self, alignment=None):
        self.dm.data=self.image.tobytes()
        return evaluate(rgb_meta=self.meta,depth_message=self.dm,camera_info=self.ci,
            binding=self.binding,alignment=alignment or self.alignment,policy=self.policy,
            rectified=True,max_info_age_ns=10**9,selected=self.selected,
            depth_m=viewer.meters(self.image,'16UC1'),sizes=[5,11,21],roi_geometry=viewer.roi_geometry)

    def test_upstream_identity(self):
        root=Path(__file__).parent
        manifest=json.loads((root/'source_manifest.json').read_text())
        for entry in manifest['files']:
            data=(root/entry['path']).read_bytes()
            sha=hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()
            self.assertEqual(sha,entry['blob_sha'])

    def test_units_and_invalids(self):
        depth=viewer.meters(np.array([[0,2000]],np.uint16),'16UC1')
        self.assertTrue(np.isnan(depth[0,0])); self.assertAlmostEqual(depth[0,1],2.)
        depth=viewer.meters(np.array([[2.,np.inf,-1.]],np.float32),'32FC1')
        self.assertEqual(depth[0,0],2.); self.assertTrue(np.isnan(depth[0,1:]).all())
        with self.assertRaises(ValueError): viewer.meters(self.image,'mono16')

    def test_same_pair_sizes_and_xyz(self):
        rows=self.rows()
        self.assertEqual([r['roi_pixels'] for r in rows],[25,121,441])
        for row in rows:
            self.assertEqual(row['status'],'LOCATED')
            self.assertTrue(np.allclose(row['point_optical'],[-.682,-.162,2.]))

    def test_background_mixture(self):
        self.image[195:206,145:156]=2700
        self.image[198:203,148:153]=2000
        self.assertEqual([r['status'] for r in self.rows()],
            ['LOCATED','DEPTH_DISCONTINUITY','DEPTH_DISCONTINUITY'])

    def test_invalid_center_patch(self):
        self.image[198:203,148:153]=0
        self.assertEqual([r['status'] for r in self.rows()],
            ['DEPTH_INSUFFICIENT','LOCATED','LOCATED'])

    def test_alignment_and_clock_not_fabricated(self):
        self.assertTrue(all(r['status']=='ALIGNMENT_UNVERIFIED' for r in self.rows(replace(self.alignment,verified=False))))
        self.meta=replace(self.meta,stamp_semantics='UNKNOWN')
        self.assertTrue(all(r['status']=='CLOCK_UNCERTAIN' for r in self.rows()))

    def test_late_depth_and_eligibility(self):
        cache=viewer.SelectiveDepth(capacity=90,slop_ns=20_000_000,wait_s=1.5,confidence=.9,label='survivor')
        self.assertTrue(cache.offer_rgb(10**10,640,480,[self.selected],'frame',0.))
        self.assertIsNone(cache.take(.7))
        cache.offer_depth(10**10+10_000_000,self.dm,.78)
        pair=cache.take(.78)
        self.assertEqual((pair[0],pair[3]),('frame',10_000_000))
        self.assertFalse(cache.offer_rgb(11*10**9,640,480,[replace(self.selected,confidence=.89)],None,1.))
        self.assertFalse(cache.offer_rgb(12*10**9,640,480,[replace(self.selected,bbox=(0.,1.,20.,20.))],None,1.))
        self.assertTrue(cache.offer_rgb(13*10**9,640,480,[self.selected],None,1.))
        cache.expire(2.6); self.assertEqual(cache.expired_requests,1)

    def test_csv_dedupe_and_known_error(self):
        with tempfile.TemporaryDirectory() as root:
            c=Collector(root,[5,11,21],'synthetic',2.,30.,{})
            c.start(); folder=Path(c.active['folder'])
            try:
                for i in range(6):
                    self.image[:]=1990 if i%2 else 2010
                    context=dict(rgb_stamp_ns=10**9+i,depth_stamp_ns=10**9+i)
                    rows=self.rows()
                    c.submit(rows,context,time.monotonic()); c.submit(rows,context,time.monotonic())
            finally: c.close()
            report=summarize(folder,plots=False)
            self.assertEqual(report['complete_pairs'],6)
            self.assertTrue(report['writer_count_matches'])
            for row in report['metrics']:
                self.assertAlmostEqual(row['mae_m'],.01)
                self.assertAlmostEqual(row['common_mae_m'],.01)
                self.assertEqual(row['passed_pairs'],6)

    def test_empty_trial_has_no_fake_zero_error(self):
        with tempfile.TemporaryDirectory() as root:
            c=Collector(root,[5,11,21],'empty',None,30.,{})
            c.start(); folder=Path(c.active['folder']); c.close()
            report=summarize(folder,plots=False)
            self.assertEqual(report['status'],'NO_MATCHED_DATA')
            for row in report['metrics']:
                self.assertIsNone(row['mae_m']); self.assertIsNone(row['pass_rate'])


if __name__=='__main__':
    unittest.main(verbosity=2)
