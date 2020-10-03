import unittest

class TestStringMethods(unittest.TestCase):
    def test_estimate_height_and_speed_knowing_liftoff_plus(self):
        self.assertEqual('foo'.upper(), 'FOO')
        # Set liftoff plus state
        # Use Hopper2dController to calculate height & speed
        # Simulate
        # Get max height & speed

    def test_set_liftoff_plus_knowing_desired_height_and_speed(self):
        self.assertEqual('foo'.upper(), 'FOO')
        # Set desired height & speed
        # Use Hopper2dController to calculate liftoff plus state
        # Simulate
        # Get max height & speed

if __name__ == '__main__':
    unittest.main(verbosity=2)
