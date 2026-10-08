import unittest
from rescue_contracts import (RescueError, CANONICAL_CODES, DETAIL_CATEGORIES,
    canonical_code, robot_ids, timestamp_ns, duration_ns, ros_time_to_ns,
    ns_to_ros_time, seconds_to_ns)


class SharedContractTests(unittest.TestCase):
    def test_real_epoch_exact_round_trip(self):
        value = 1700000000123456789
        self.assertEqual(ros_time_to_ns(*ns_to_ros_time(value)), value)
        self.assertEqual(ns_to_ros_time(value), (1700000000, 123456789))

    def test_no_float_or_boolean_timestamp(self):
        for value in (True, 1.0, float('nan'), 0, -1, 2**63):
            with self.assertRaises(RescueError): timestamp_ns(value)
        self.assertEqual(timestamp_ns(2**63-1), 2**63-1)
        self.assertEqual(duration_ns(0), 0)

    def test_ros_header_bounds(self):
        for sec, nano in ((True, 1), (1, True), (-1, 0), (2**31, 0), (1, -1), (1, 10**9), (0, 0)):
            with self.assertRaises(RescueError): ros_time_to_ns(sec, nano)
        with self.assertRaises(RescueError): ns_to_ros_time(2**63-1)

    def test_decimal_policy_conversion_is_exact(self):
        self.assertEqual(seconds_to_ns('0.000000001'), 1)
        self.assertEqual(seconds_to_ns(.2), 200000000)
        for value in ('0.0000000001', True, 'NaN', 'Infinity', '-1', '1e10000'):
            with self.assertRaises(RescueError): seconds_to_ns(value)

    def test_two_configured_ids_no_defaults(self):
        self.assertEqual(robot_ids(['bravo', 'alpha']), ('alpha', 'bravo'))
        for values in (['a'], ['a','a'], ['a','b','c'], ['a','bad-id'], 'ab', None):
            with self.assertRaises(RescueError): robot_ids(values)

    def test_component_dictionary_has_only_canonical_targets(self):
        for component, codes in DETAIL_CATEGORIES.items():
            for detail, code in codes.items():
                self.assertIn(code, CANONICAL_CODES)
                self.assertEqual(canonical_code(component, detail), code)
        error = RescueError('STALE_TRANSFORM', 'query stamp mismatch',
                            component='rescue_mapping', detail_code='TF_TIME_MISMATCH')
        self.assertEqual(error.detail_code, 'rescue_mapping.TF_TIME_MISMATCH')
        with self.assertRaises(ValueError): canonical_code('rescue_mapping', 'UNREGISTERED_CODE')
