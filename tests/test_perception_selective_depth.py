import unittest
from types import SimpleNamespace as S
from rescue_perception.selective_depth import SelectiveDepth

def detection(conf=.95,box=(1,1,9,9),label='survivor'):
    return S(confidence=conf,bbox=box,class_label=label)

class SelectiveDepthTests(unittest.TestCase):
    def gate(self):return SelectiveDepth(capacity=3,slop_ns=20,wait_s=.5,confidence=.9,label='survivor')
    def offer(self,g,stamp=100,outputs=None,now=0):
        return g.offer_rgb(stamp,10,10,outputs if outputs is not None else [detection()], 'rgb-payload',now)
    def test_empty_low_confidence_and_clipped_boxes_do_not_request_depth(self):
        for outputs in ([],[detection(.89)],[detection(box=(0,1,9,9))],[detection(label='person')]):
            g=self.gate();g.offer_depth(100,'depth',0)
            self.assertFalse(self.offer(g,outputs=outputs));self.assertIsNone(g.take(0))
    def test_depth_before_or_after_detection_is_matched_by_original_stamp(self):
        for early in (True,False):
            g=self.gate()
            if early:g.offer_depth(105,'depth',0)
            self.assertTrue(self.offer(g))
            if not early:g.offer_depth(105,'depth',.1)
            result=g.take(.1)
            self.assertEqual((result[0],result[2],result[3]),('rgb-payload','depth',5))
            self.assertIsNone(g.take(.1))
    def test_one_best_detection_and_nearest_depth_only(self):
        g=self.gate();g.offer_depth(115,'far',0);g.offer_depth(101,'near',0)
        best=detection(.99);self.offer(g,outputs=[detection(.95),best])
        payload,selected,depth,delta=g.take(0)
        self.assertIs(selected,best);self.assertEqual(depth,'near')
    def test_wait_is_bounded_and_failed_attempt_can_retry_next_rgb(self):
        g=self.gate();self.offer(g);g.offer_depth(121,'out-of-slop',0)
        self.assertIsNone(g.take(.1));self.assertIsNone(g.take(.5))
        self.assertTrue(self.offer(g,stamp=200,now=.6));g.offer_depth(201,'valid',.6)
        self.assertIsNotNone(g.take(.6))
    def test_buffer_and_pending_rgb_are_bounded(self):
        g=self.gate()
        for stamp in range(10):g.offer_depth(stamp,str(stamp),0)
        self.assertEqual(g.buffered_count,3)
        self.offer(g);self.assertFalse(self.offer(g,stamp=101,now=.1))
        g.offer_depth(100,'match',.1)
        self.assertEqual(g.take(.1)[0],'rgb-payload')
    def test_sent_disables_all_depth_work(self):
        g=self.gate();self.offer(g);g.mark_sent();g.offer_depth(100,'depth',0)
        self.assertFalse(self.offer(g,stamp=200));self.assertIsNone(g.take(0));self.assertEqual(g.buffered_count,0)
    def test_duplicate_or_reordered_rgb_does_not_rearm(self):
        g=self.gate();self.offer(g);g.offer_depth(100,'depth',0);g.take(0)
        self.assertFalse(self.offer(g,stamp=100));self.assertFalse(self.offer(g,stamp=99))
    def test_expired_depth_is_not_reused(self):
        g=self.gate();g.offer_depth(100,'stale',0);self.offer(g,now=.6)
        self.assertIsNone(g.take(.6))
