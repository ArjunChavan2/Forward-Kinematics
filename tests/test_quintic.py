"""quintic.py boundary conditions. Skipped until coefficients/evaluate are implemented."""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
import quintic  # noqa: E402


def solve(*args):
    try:
        return quintic.coefficients(*args)
    except NotImplementedError:
        raise unittest.SkipTest("quintic.py not implemented yet")


class QuinticTest(unittest.TestCase):
    def assertState(self, got, want):
        for g, w in zip(got, want):
            self.assertAlmostEqual(g, w, places=9)

    def test_hits_all_six_boundary_conditions(self):
        start, end, T = (0.3, -1.2, 4.0), (2.0, 0.5, -3.0), 0.7
        c = solve(*start, *end, T)
        self.assertEqual(len(c), 6)
        self.assertState(quintic.evaluate(c, 0.0), start)
        self.assertState(quintic.evaluate(c, T), end)

    def test_rest_to_rest_passes_the_midpoint_at_half_time_at_peak_speed(self):
        c = solve(-1.0, 0.0, 0.0, 3.0, 0.0, 0.0, 2.0)
        q, v, a = quintic.evaluate(c, 1.0)
        self.assertAlmostEqual(q, 1.0, places=9)
        self.assertAlmostEqual(a, 0.0, places=9)
        self.assertAlmostEqual(v, 15 / 8 * 4.0 / 2.0, places=9)  # peak speed of a rest-to-rest quintic: 15/8 * distance / T

    def test_stays_put_when_start_equals_end(self):
        c = solve(0.5, 0.0, 0.0, 0.5, 0.0, 0.0, 1.0)
        for t in (0.0, 0.25, 0.5, 1.0):
            self.assertState(quintic.evaluate(c, t), (0.5, 0.0, 0.0))
        self.assertTrue(all(math.isfinite(x) for x in c))


if __name__ == "__main__":
    unittest.main()
