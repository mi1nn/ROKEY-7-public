"""Coordinator -> fixed webcam watch (SetCameraWatch, IF-078) without ROS (E2).

C-11 rows 1-4: the coordinator issues ``watch_id`` on the operator's SET_CAMERA_WATCH, records it
when it sends the request and drops it when the camera refuses; one active watch per camera; a
candidate is accepted only for the active watch with the same camera and target and a declared
time basis, and any candidate ends that watch. #124: only in DISASTER, and the watch is released
when the mission leaves it. The ROI comes from the request or the coordinator parameter; there is
no full-frame default.
"""
import unittest

from rescue_contracts import RescueError
from rescue_coordination import ros_node
from rescue_coordination.watch import CameraWatchBook, roi_value
from test_coordination_ros_core import location, make_core, receipt, request, site_msg, time_msg
from test_coordination_search import NOW, uid

CAMERA = 'cam_fixed'
ROI = (.1, .2, .5, .6)
TIMEOUT = 2_000_000_000


def book(roi=ROI, ids=None):
    ids = ids or iter(range(5000, 6000))
    return CameraWatchBook(CAMERA, roi=roi, request_timeout_ns=TIMEOUT, new_id=lambda: uid(next(ids)))


def reply(fields, accepted=True, revision=None, reason='ACCEPTED'):
    rev = revision if revision is not None else fields['expected_watch_revision'] + 1
    return dict(watch_id=fields['watch_id'], accepted=accepted, watch_revision=rev,
                reason_code=reason if accepted or reason != 'ACCEPTED' else 'REVISION_MISMATCH')


def candidate(n, watch_id, target_id, **changes):
    values = dict(kind='TARGET_DISAPPEARED_CANDIDATE', target_id=target_id, watch_id=watch_id,
                  event_id=uid(350 + n))
    return site_msg(n, **(values | changes))


def changed(msg, **changes):
    return type(msg)(**(vars(msg) | changes))


class WatchTestCase(unittest.TestCase):
    def setUp(self):
        self.core = make_core(camera_watch=book())
        self.core.on_site_event(site_msg())
        self.target = self.core.on_target_location(location(1), now_ns=NOW)
        (report,) = self.core.reports_due(NOW)
        self.core.on_receipt(receipt(report))

    def arm(self, n=1, now=NOW, **fields):
        reply_fields = self.core.on_command(request('SET_CAMERA_WATCH', command_id=uid(720 + n),
                                                    target_id=fields.pop('target_id', self.target), **fields),
                                            now_ns=now)
        return reply_fields, self.core.watch_requests(now)

    def armed(self):
        answer, (fields,) = self.arm()
        self.assertTrue(answer['accepted'])
        self.assertTrue(self.core.on_watch_reply(**reply(fields)))
        return fields


class ArmTests(WatchTestCase):
    def test_set_camera_watch_issues_one_request_with_a_new_watch_id(self):
        answer, (fields,) = self.arm()
        self.assertEqual((True, 'ACCEPTED'), (answer['accepted'], answer['request_state']))
        self.assertEqual((CAMERA, self.target, ROI, True, 0, 'OPERATOR_ROI', uid(721), NOW, NOW + TIMEOUT),
                         (fields['camera_id'], fields['target_id'], fields['roi_xyxy_normalized'], fields['enable'],
                          fields['expected_watch_revision'], fields['association_method'],
                          fields['association_evidence_id'], fields['issued_at_ns'], fields['deadline_ns']))
        self.assertNotIn(fields['watch_id'], (self.target, uid(721)))
        self.assertNotEqual(fields['command_id'], uid(721))  # the camera caches replies by its own command_id
        self.assertIsNone(self.core.camera_watch.active)  # recorded, not active until the camera accepts
        self.assertEqual([], self.core.watch_requests(NOW))

    def test_camera_acceptance_makes_it_the_active_watch(self):
        fields = self.armed()
        active = self.core.camera_watch.active
        self.assertEqual((fields['watch_id'], self.target, ROI, 1),
                         (active.watch_id, active.target_id, active.roi, self.core.camera_watch.revision))

    def test_camera_refusal_drops_the_watch_and_asks_to_arm_again(self):
        _, (fields,) = self.arm()
        self.assertTrue(self.core.on_watch_reply(**reply(fields, accepted=False, revision=4,
                                                         reason='REVISION_MISMATCH')))
        watch = self.core.camera_watch
        self.assertEqual((None, 4, False), (watch.active, watch.revision, fields['watch_id'] in watch.bindings))
        self.assertEqual('WATCH_REARM_NEEDED', self.core.system_state_change(NOW)['reason'])
        _, (again,) = self.arm(2)  # the next request carries the revision the camera reported
        self.assertEqual(4, again['expected_watch_revision'])

    def test_a_reply_for_another_watch_is_ignored(self):
        _, (fields,) = self.arm()
        self.assertFalse(self.core.on_watch_reply(**(reply(fields) | dict(watch_id=uid(999)))))
        self.assertIsNone(self.core.camera_watch.active)

    def test_no_reply_before_the_deadline_drops_the_request(self):
        _, (fields,) = self.arm()
        self.core.tick(NOW + TIMEOUT)
        self.assertIsNotNone(self.core.camera_watch.pending)
        self.core.tick(NOW + TIMEOUT + 1)
        self.assertIsNone(self.core.camera_watch.pending)
        self.assertEqual('WATCH_REARM_NEEDED', self.core.camera_watch.rearm)
        self.assertFalse(self.core.on_watch_reply(**reply(fields)))  # too late: not made active

    def test_a_new_watch_replaces_the_active_one_on_the_same_camera(self):
        first = self.armed()
        answer, (second,) = self.arm(2, roi_xyxy_normalized=[.2, .2, .4, .4])
        self.assertEqual((True, 1, (.2, .2, .4, .4)), (answer['accepted'], second['expected_watch_revision'],
                                                        second['roi_xyxy_normalized']))
        self.assertNotEqual(first['watch_id'], second['watch_id'])
        self.core.on_watch_reply(**reply(second))
        self.assertEqual(second['watch_id'], self.core.camera_watch.active.watch_id)


class ArmGateTests(WatchTestCase):
    def test_only_in_disaster_mode(self):
        core = make_core(camera_watch=book())
        reply_fields = core.on_command(request('SET_CAMERA_WATCH', command_id=uid(721), target_id=uid(500)),
                                       now_ns=NOW)
        self.assertEqual((False, 'NOT_AUTHORIZED'), (reply_fields['accepted'], reply_fields['reason_code']))
        self.assertEqual([], core.watch_requests(NOW))

    def test_target_must_be_known_and_not_judged(self):
        answer, sent = self.arm(target_id=uid(999))
        self.assertEqual(('NOT_FOUND', []), (answer['reason_code'], sent))
        self.core.mission.open_rechecks[self.target] = None
        self.core.mission.record_rescue_verdict(self.target, 'RESCUE_CONFIRMED')
        answer, sent = self.arm(2)
        self.assertEqual(('INVALID_INPUT', []), (answer['reason_code'], sent))
        self.assertEqual('TARGET_ALREADY_JUDGED', self.core.last_reason)

    def test_no_roi_anywhere_is_refused_not_full_frame(self):
        self.core = make_core(camera_watch=book(roi=None))
        self.core.on_site_event(site_msg())
        self.target = self.core.on_target_location(location(1), now_ns=NOW)
        answer, sent = self.arm()
        self.assertEqual((False, 'PENDING_EVIDENCE', []), (answer['accepted'], answer['reason_code'], sent))
        self.assertEqual('NO_WATCH_ROI', self.core.last_reason)
        answer, (fields,) = self.arm(2, roi_xyxy_normalized=[.3, .3, .7, .9])  # the operator's ROI
        self.assertEqual((.3, .3, .7, .9), fields['roi_xyxy_normalized'])

    def test_bad_roi_is_invalid_input(self):
        for n, roi in enumerate(([.5, .2, .1, .6], [.1, .2, 1.5, .6], [float('nan'), 0., 1., 1.]), 1):
            with self.subTest(roi=roi):
                answer, sent = self.arm(n, roi_xyxy_normalized=roi)
                self.assertEqual(('INVALID_INPUT', []), (answer['reason_code'], sent))
                self.assertEqual('BAD_WATCH_ROI', self.core.last_reason)

    def test_no_target_watch_camera_in_the_profile(self):
        self.core = make_core()  # camera_watch=None
        self.core.on_site_event(site_msg())
        answer, sent = self.arm(target_id=uid(500))
        self.assertEqual(('NOT_FOUND', []), (answer['reason_code'], sent))
        self.assertEqual('NO_WATCH_CAMERA', self.core.last_reason)

    def test_one_request_in_flight(self):
        self.arm()
        answer, sent = self.arm(2)
        self.assertEqual(('TASK_BUSY', []), (answer['reason_code'], sent))
        self.assertEqual('WATCH_PENDING', self.core.last_reason)

    def test_replay_answers_duplicate_and_sends_nothing(self):
        self.arm()
        answer, sent = self.arm()
        self.assertEqual(('DUPLICATE', True, []), (answer['request_state'], answer['accepted'], sent))
        conflict = self.core.on_command(request('SET_CAMERA_WATCH', command_id=uid(721), target_id=self.target,
                                                roi_xyxy_normalized=[.2, .2, .3, .3]), now_ns=NOW)
        self.assertEqual('ID_CONTENT_CONFLICT', conflict['reason_code'])


class CandidateTests(WatchTestCase):
    def test_linked_candidate_is_accepted_and_ends_the_watch(self):
        fields = self.armed()
        self.assertEqual('CANDIDATE_ACCEPTED',
                         self.core.on_site_event(candidate(1, fields['watch_id'], self.target)))
        self.assertEqual(uid(351), self.core.camera_watch.accepted[self.target].event_id)
        self.core.tick(NOW)
        (release,) = self.core.watch_requests(NOW)
        self.assertEqual((False, fields['watch_id'], ROI, 1),
                         (release['enable'], release['watch_id'], release['roi_xyxy_normalized'],
                          release['expected_watch_revision']))
        self.core.on_watch_reply(**reply(release))
        self.assertIsNone(self.core.camera_watch.active)
        self.assertIsNone(self.core.camera_watch.rearm)  # accepted: nothing to re-arm

    def test_candidate_replay_is_idempotent_and_conflicts_are_rejected(self):
        fields = self.armed()
        self.core.on_site_event(candidate(1, fields['watch_id'], self.target))
        self.assertEqual('CANDIDATE_ACCEPTED', self.core.on_site_event(candidate(1, fields['watch_id'], self.target)))
        with self.assertRaises(RescueError) as caught:  # another event_id for the same watch_id (row 3)
            self.core.on_site_event(candidate(2, fields['watch_id'], self.target))
        self.assertEqual('ID_CONTENT_CONFLICT', caught.exception.code)

    def test_unlinked_or_untimed_candidates_are_refused_and_end_the_watch(self):
        cases = (('other target', dict(target_id=uid(999)), 'CANDIDATE_NOT_LINKED'),
                 ('unknown watch', dict(watch_id=uid(998)), 'CANDIDATE_NOT_LINKED'),
                 ('no time basis', dict(time=time_msg(clock_state='UNKNOWN', has_clock_error_bound=False)),
                  'CANDIDATE_TIME_BASIS_UNUSABLE'))
        for n, (label, changes, reason) in enumerate(cases, 1):
            with self.subTest(label):
                self.setUp()
                fields = self.armed()
                msg = changed(candidate(n, fields['watch_id'], self.target), **changes)
                self.assertEqual(reason, self.core.on_site_event(msg))
                self.assertNotIn(self.target, self.core.camera_watch.accepted)
                self.core.tick(NOW)
                release = self.core.watch_requests(NOW)
                ended = changes.get('watch_id') is None
                self.assertEqual(ended, bool(release))  # only the watch the candidate names ends
                self.assertEqual('WATCH_REARM_NEEDED', self.core.camera_watch.rearm)

    def test_candidate_outside_disaster_is_recorded_only(self):
        core = make_core(camera_watch=book())
        self.assertEqual('CANDIDATE_NOT_DISASTER', core.on_site_event(candidate(1, uid(5000), uid(500))))
        self.assertEqual({}, core.camera_watch.accepted)


class ReleaseTests(WatchTestCase):
    def released(self, now=NOW):
        self.core.tick(now)
        return self.core.watch_requests(now)

    def test_no_release_while_the_mission_runs(self):
        self.armed()
        self.assertEqual([], self.released())

    def test_released_when_the_target_is_judged(self):
        fields = self.armed()
        self.core.mission.open_rechecks[self.target] = None
        self.core.mission.record_rescue_verdict(self.target, 'RESCUE_CONFIRMED')
        (release,) = self.released()
        self.assertEqual((False, fields['watch_id']), (release['enable'], release['watch_id']))
        self.assertEqual([], self.released())  # one release in flight

    def test_released_when_the_mode_is_not_disaster(self):
        self.armed()
        self.core.mission.mode = 'NORMAL'  # no path back to NORMAL yet; #124 asks for the release when there is
        (release,) = self.released()
        self.assertFalse(release['enable'])

    def test_released_when_returning(self):
        self.armed()
        self.core.mission.returning = True
        (release,) = self.released()
        self.assertFalse(release['enable'])

    def test_release_with_a_stale_revision_is_retried_with_the_camera_revision(self):
        self.armed()
        self.core.mission.mode = 'NORMAL'
        (release,) = self.released()
        self.core.on_watch_reply(**reply(release, accepted=False, revision=7, reason='REVISION_MISMATCH'))
        self.assertIsNotNone(self.core.camera_watch.active)
        (again,) = self.released()
        self.assertEqual(7, again['expected_watch_revision'])
        self.core.on_watch_reply(**reply(again, accepted=False, revision=8, reason='WATCH_MISMATCH'))
        self.assertIsNone(self.core.camera_watch.active)  # the camera does not hold it: nothing left to release
        self.assertEqual([], self.released())


class ValueTests(unittest.TestCase):
    def test_roi_value(self):
        self.assertEqual(ROI, roi_value(list(ROI)))
        self.assertIsNone(roi_value([0.] * 4))  # the IDL's "not given"

        class Float64(float):  # what a ROS float64[4] field yields (numpy.float64 subclasses float)
            pass
        self.assertEqual(ROI, roi_value([Float64(v) for v in ROI]))
        self.assertEqual(ROI, roi_value(tuple(Float64(v) for v in ROI)))
        with self.assertRaises(RescueError):
            roi_value([True, 0., 1., 1.])
        for bad in ([0., 0., 1.], [.5, .5, .5, .9], [-.1, 0., 1., 1.], ['a', 0., 1., 1.]):
            with self.subTest(bad=bad), self.assertRaises(RescueError):
                roi_value(bad)

    def test_build_core_takes_the_single_target_watch_camera(self):
        self.assertEqual('/rescue/cameras/cam_fixed/set_watch', ros_node.camera_watch_service('cam_fixed'))


if __name__ == '__main__':
    unittest.main()
