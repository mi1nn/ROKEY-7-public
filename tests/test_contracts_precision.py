import unittest
from decimal import localcontext
from rescue_contracts import RescueError, seconds_to_ns


class DecimalPrecisionRegressionTests(unittest.TestCase):
    def test_subnanosecond_fraction_cannot_round_to_integer(self):
        with self.assertRaises(RescueError):
            seconds_to_ns('1.00000000000000000000000000001')

    def test_ambient_decimal_precision_cannot_erase_epoch_nanosecond(self):
        with localcontext() as context:
            context.prec = 9
            self.assertEqual(seconds_to_ns('1700000000.000000001'), 1700000000000000001)

    def test_trailing_zero_and_huge_exponents(self):
        self.assertEqual(seconds_to_ns('1.00000000000000000000000000000'), 1000000000)
        self.assertEqual(seconds_to_ns('0e1000000'), 0)
        for value in ('1e1000000', '1e-1000000'):
            with self.assertRaises(RescueError): seconds_to_ns(value)
