"""Wire validation using ROS-shaped objects; no ROS, authority or field policy."""
import copy
import unittest
from types import SimpleNamespace as NS

from rescue_robot.executor import ContractError, GoalPose
from rescue_robot.task_wire import validate_task_wire, validate_resolved_goal, validate_completion_scope
from test_task_goal_fields import uid

NOW = 1_700_000_000_000_000_000
SECOND = 1_000_000_000


def frame():
    return NS(robot_id='robot_a', native_frame='robot_a/map', mission_frame='mission',
              map_session_id=uid(6), map_revision=0, transform_revision=1, alignment_status='VALID')


def wire(kind='MOVE_SEGMENT'):
    targeted = kind in ('MOVE_SEGMENT', 'APPROACH_TARGET', 'REOBSERVE_TARGET')
    goal = NS(command_id=uid(3), task_id=uid(4), robot_id='robot_a', expected_robot_boot_id=uid(1),
              coordinator_id='coordinator', authority_epoch=uid(2), generation=1, kind=kind,
              issued_at_ns=NOW, deadline_ns=NOW+10*SECOND, lease_id=uid(8),
              target_id=uid(7) if targeted else '', target_estimate_revision=0,
              search_area_ref=uid(9) if kind=='SEARCH_AREA' else '',
              home_ref=uid(10) if kind=='RETURN_HOME' else '', frame_ref=frame(),
              has_route=kind=='MOVE_SEGMENT', has_approval=kind=='MOVE_SEGMENT', policy_version='test-only')
    goal.route = NS(route_id=uid(11), route_revision=2, robot_id='robot_a', target_id=goal.target_id,
                    segment_index=3, frame_ref=frame(), policy_version='test-only',
                    generated_at_ns=NOW, valid_until_ns=NOW+5*SECOND)
    goal.approval = NS(approval_id=uid(12), command_id=uid(13), operator_id='operator',
                       robot_id='robot_a', robot_boot_id=uid(1), authority_epoch=uid(2),
                       target_id=uid(7), target_estimate_revision=0, route_id=uid(11), route_revision=2,
                       segment_index=3, scope='ONE_SEGMENT', approved_at_ns=NOW, valid_until_ns=NOW+5*SECOND)
    lease = NS(lease_id=uid(8), robot_id='robot_a', robot_boot_id=uid(1), coordinator_id='coordinator',
               authority_epoch=uid(2), generation=1, renewal_sequence=0,
               issued_at_ns=NOW, valid_until_ns=NOW+2*SECOND)
    return goal, lease


def validate(goal, lease, bound=0, now=NOW):
    return validate_task_wire(goal, lease, now_ns=now, clock_error_bound_ns=bound)


class TaskWireTests(unittest.TestCase):
    def test_six_kinds_have_valid_distinct_wire_shapes(self):
        for kind in ('SEARCH_AREA','APPROACH_TARGET','REOBSERVE_TARGET','HOLD','MOVE_SEGMENT','RETURN_HOME'):
            self.assertEqual(validate(*wire(kind)),2.)

    def test_unknown_kind_or_nonboolean_flags_are_rejected(self):
        for name, value in (('kind','DOCK'),('has_route',1),('has_approval',1)):
            goal, lease=wire();setattr(goal,name,value)
            with self.assertRaises(ContractError):validate(goal,lease)

    def test_old_boot_other_robot_and_authority_are_rejected(self):
        for name,value in (('robot_id','robot_b'),('robot_boot_id',uid(20)),
                           ('authority_epoch',uid(21)),('generation',2),('lease_id',uid(22)),
                           ('coordinator_id','other')):
            goal,lease=wire();setattr(lease,name,value)
            with self.assertRaises(ContractError):validate(goal,lease)

    def test_empty_or_noncanonical_ids_are_rejected(self):
        for name in ('command_id','task_id','expected_robot_boot_id','authority_epoch','lease_id','target_id'):
            goal,lease=wire();setattr(goal,name,'')
            with self.assertRaises(ContractError):validate(goal,lease)

    def test_boolean_lease_generation_is_not_equal_to_one(self):
        goal,lease=wire();lease.generation=True
        with self.assertRaises(ContractError):validate(goal,lease)

    def test_boolean_approval_revision_and_segment_are_rejected(self):
        for name in ('target_estimate_revision','route_revision','segment_index'):
            goal,lease=wire();setattr(goal.approval,name,True)
            with self.assertRaises(ContractError):validate(goal,lease)

    def test_target_revision_cannot_be_boolean(self):
        goal,lease=wire();goal.target_estimate_revision=True
        with self.assertRaises(ContractError):validate(goal,lease)

    def test_return_requires_home_record_and_forbids_target(self):
        for name,value in (('home_ref',''),('target_id',uid(20)),('target_estimate_revision',1),
                           ('search_area_ref',uid(21))):
            goal,lease=wire('RETURN_HOME');setattr(goal,name,value)
            with self.assertRaises(ContractError):validate(goal,lease)

    def test_named_home_reference_is_valid_without_an_invented_coordinate(self):
        goal,lease=wire('RETURN_HOME');goal.home_ref='registered-home-record'
        self.assertEqual(validate(goal,lease),2.)

    def test_segment_requires_both_route_and_approval(self):
        for name in ('has_route','has_approval'):
            goal,lease=wire();setattr(goal,name,False)
            with self.assertRaises(ContractError):validate(goal,lease)

    def test_approval_must_match_all_segment_identity_fields(self):
        for name,value in (('robot_id','robot_b'),('robot_boot_id',uid(20)),('authority_epoch',uid(21)),
                           ('target_id',uid(22)),('target_estimate_revision',1),('route_id',uid(23)),
                           ('route_revision',3),('segment_index',4),('scope','ALL_SEGMENTS')):
            goal,lease=wire();setattr(goal.approval,name,value)
            with self.assertRaises(ContractError):validate(goal,lease)

    def test_approval_and_route_expiry_are_rejected_at_boundary(self):
        for field in ('approval','route'):
            goal,lease=wire();getattr(goal,field).valid_until_ns=NOW
            with self.assertRaises(ContractError):validate(goal,lease)

    def test_clock_uncertainty_is_subtracted_not_used_to_extend_lease(self):
        self.assertAlmostEqual(validate(*wire(),bound=100_000_000),1.9)
        with self.assertRaises(ContractError):validate(*wire(),bound=2*SECOND)

    def test_unmeasured_or_negative_clock_bound_is_rejected(self):
        for bound in (None,-1,True,float('inf')):
            with self.assertRaises(ContractError):validate(*wire(),bound=bound)

    def test_future_task_and_lease_are_rejected(self):
        for field in ('goal','lease'):
            goal,lease=wire();setattr(goal if field=='goal' else lease,'issued_at_ns',NOW+1)
            with self.assertRaises(ContractError):validate(goal,lease)

    def test_expired_or_invalid_intervals_are_rejected(self):
        for field in ('goal','lease'):
            goal,lease=wire();setattr(goal if field=='goal' else lease,
                                    'deadline_ns' if field=='goal' else 'valid_until_ns',NOW)
            with self.assertRaises(ContractError):validate(goal,lease)

    def test_deadline_shorter_than_lease_wins(self):
        goal,lease=wire();goal.deadline_ns=NOW+SECOND
        self.assertEqual(validate(goal,lease),1.)

    def test_route_frame_and_policy_must_match_task(self):
        for change in ('frame','policy','target','robot','segment'):
            goal,lease=wire()
            if change=='frame':goal.route.frame_ref.map_revision=1
            elif change=='policy':goal.route.policy_version='other'
            elif change=='target':goal.route.target_id=uid(20)
            elif change=='robot':goal.route.robot_id='robot_b'
            else:goal.route.segment_index=True
            with self.assertRaises(ContractError):validate(goal,lease)

    def test_alignment_unknown_blocks_motion(self):
        goal,lease=wire('RETURN_HOME');goal.frame_ref.alignment_status='UNKNOWN'
        with self.assertRaises(ContractError):validate(goal,lease)

    def test_hold_cannot_carry_move_permission(self):
        goal,lease=wire('HOLD');goal.has_route=True
        with self.assertRaises(ContractError):validate(goal,lease)

    def test_no_mutation_of_transport_payload(self):
        goal,lease=wire();before=copy.deepcopy((goal,lease));validate(goal,lease)
        self.assertEqual((goal,lease),before)

    def test_segment_endpoint_is_not_inferred_from_final_approach(self):
        goal,_=wire();pose=GoalPose('mission',3,4,0,uid(6),0,1)
        self.assertIs(validate_resolved_goal(goal,pose),pose)

    def test_resolver_must_preserve_frame_and_revision(self):
        goal,_=wire('RETURN_HOME')
        for pose in (None,GoalPose('robot_a/map',0,0,0,uid(6),0,1),
                     GoalPose('mission',0,0,0,uid(6),1,1)):
            with self.assertRaises(ContractError):validate_resolved_goal(goal,pose)

    def test_hold_has_no_destination_pose(self):
        goal,_=wire('HOLD');self.assertIsNone(validate_resolved_goal(goal,None))
        with self.assertRaises(ContractError):
            validate_resolved_goal(goal,GoalPose('mission',0,0,0,uid(6),0,1))

    def test_segment_result_cannot_be_reported_as_target_approach(self):
        for scope in ('TARGET_APPROACH','RECHECK','RETURN','SEARCH'):
            with self.assertRaises(ContractError):
                validate_completion_scope('MOVE_SEGMENT',NS(completion_scope=scope))
        self.assertEqual(validate_completion_scope('MOVE_SEGMENT',NS(completion_scope='SEGMENT')),
                         NS(completion_scope='SEGMENT'))

    def test_return_result_has_return_scope(self):
        with self.assertRaises(ContractError):
            validate_completion_scope('RETURN_HOME',NS(completion_scope='TARGET_APPROACH'))


if __name__ == '__main__':
    unittest.main()
