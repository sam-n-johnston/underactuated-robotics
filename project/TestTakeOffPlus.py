import unittest
import numpy as np
from hopper_2d import Simulate2dHopper

class TestTakeOffPlus(unittest.TestCase):
    def test_estimate_z_and_xd_knowing_liftoff_plus(self):
        lift_off_plus_state = np.zeros(10)
        lift_off_plus_state[1] = 1.51 # foot just touching the ground
        lift_off_plus_state[4] = 0.5 # l distance
        lift_off_plus_state[0+5] = 0.5 # xd, horizontal speed
        lift_off_plus_state[1+5] = 0.5 # z, vertical height

        # Use Simulate2dHopper to simulate
        hopper, controller, state_log = Simulate2dHopper(x0 = lift_off_plus_state,
                               duration=2,
                               desired_lateral_velocity = 0.5)

        # Find first apex in simulation
        simulated_zs = state_log.data()[1, :]
        index = 1 # Skip first time step because simulation is strange
        first_apex_index = -1

        while index < len(simulated_zs) and first_apex_index == -1:
            index = index + 1
            if  simulated_zs[index] > simulated_zs[index - 1] and \
                simulated_zs[index] > simulated_zs[index + 1]:
                first_apex_index = index

        # Get simulated max height (z) & horizontal speed (xd) at first apex
        simulated_xds = state_log.data()[0+5, :]
        simulated_max_z = simulated_zs[first_apex_index]
        simulated_max_xd = simulated_xds[first_apex_index]

        # Use Hopper2dController to get max height (z) & speed (xd) based on liftoff state
        calculated_max_z = controller.calculate_apex_z_based_off_liftoff_plus(lift_off_plus_state)
        calculated_max_xd = controller.calculate_apex_xd_based_off_liftoff_plus(lift_off_plus_state)

        # Compare both values
        print('Simulated vs calculated max height (z): ' + \
            '%.2f' % simulated_max_z + ' VS ' + '%.2f' % calculated_max_z)
        print('Simulated vs calculated max horizontal speed (xd): ' + \
            '%.2f' % simulated_max_xd + ' VS ' + '%.2f' % calculated_max_xd)

        self.assertAlmostEqual(simulated_max_z, calculated_max_z, 2)
        self.assertAlmostEqual(simulated_max_xd, calculated_max_xd, 2)

    def test_set_liftoff_plus_knowing_desired_height_and_speed(self):
        self.assertEqual('foo'.upper(), 'FOO')
        # Set desired height & speed
        # Use Hopper2dController to calculate liftoff plus state
        # Simulate
        # Get max height & speed

if __name__ == '__main__':
    unittest.main(verbosity=2)
