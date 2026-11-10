import copy
import math
import sys
from pathlib import Path
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'nodes'))
from sonar_nav2_compat import finite_no_echo  # noqa: E402


def observation(value):
    return SimpleNamespace(range=value, field_of_view=.52, min_range=.02,
                           max_range=4., radiation_type=0, variance=0.,
                           header=SimpleNamespace(frame_id='sonar_right',
                                                  stamp=(123, 456)))


class NoEcho(unittest.TestCase):
    def test_no_return_preserves_raw_and_capture_metadata(self):
        raw = observation(math.inf)
        before = copy.deepcopy(raw)
        result = finite_no_echo(raw)
        self.assertEqual(raw, before)
        self.assertIsNot(result, raw)
        self.assertEqual(result.range, 4.)
        result.range = math.inf
        self.assertEqual(result, raw)

    def test_near_echo_and_out_of_range_echo_unchanged(self):
        for value in (.02, .05, .1, .5, 4., 4.1):
            raw = observation(value)
            self.assertIs(finite_no_echo(raw), raw)

    def test_invalid_values_are_not_reinterpreted_as_free_space(self):
        for value in (math.nan, -math.inf):
            raw = observation(value)
            self.assertIs(finite_no_echo(raw), raw)
        for field, value in [('min_range', -.1), ('max_range', .01),
                             ('max_range', math.inf), ('field_of_view', 0.),
                             ('field_of_view', 3.2), ('field_of_view', math.nan)]:
            raw = observation(math.inf)
            setattr(raw, field, value)
            self.assertIs(finite_no_echo(raw), raw)


if __name__ == '__main__':
    unittest.main()
