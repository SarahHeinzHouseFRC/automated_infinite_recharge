import geometry
import numpy as np
import unittest


def sign(value):
    if value > 0:
        return +1
    elif value < 0:
        return -1
    return 0


class TestGeometryModule(unittest.TestCase):
    def setUp(self):
        self.p1 = np.array((0, 0))
        self.p2 = np.array((0, 1))
        self.p3 = np.array((0, 2))
        self.p4 = np.array((1, 0))

        self.p1_to_p2 = self.p2 - self.p1
        self.p1_to_p3 = self.p3 - self.p1
        self.p1_to_p4 = self.p4 - self.p1

    def test_ccw_returns_zero_when_points_are_collinear(self):
        # Act.
        expected_ccw_value = sign(np.cross(self.p1_to_p2, self.p1_to_p3))
        actual_ccw_value = sign(geometry.ccw(self.p1, self.p2, self.p3))

        # Assert.
        self.assertEqual(expected_ccw_value, actual_ccw_value)

    def test_ccw_returns_positive_value_when_angle_formed_has_positive_orientation(
        self,
    ):
        # Act.
        expected_sign = sign(np.cross(self.p1_to_p2, self.p1_to_p4))
        actual_sign = sign(geometry.ccw(self.p2, self.p1, self.p4))

        # Assert.
        self.assertEqual(expected_sign, actual_sign)

    def test_ccw_returns_negative_value_when_angle_formed_has_negative_orientation(
        self,
    ):
        # Act.
        expected_sign = sign(np.cross(self.p1_to_p4, self.p1_to_p2))
        actual_sign = sign(geometry.ccw(self.p4, self.p1, self.p2))

        # Assert.
        self.assertEqual(expected_sign, actual_sign)


if __name__ == "__main__":
    unittest.main()
