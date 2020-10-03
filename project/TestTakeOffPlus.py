import unittest
import numpy as np
from hopper_2d import Simulate2dHopper

class TestTakeOffPlus(unittest.TestCase):
    def test_apedx_z_and_xd_based_off_liftoff_plus_1(self):
        lift_off_plus_state = np.zeros(10)
        lift_off_plus_state[1] = 1.51 # foot just touching the ground
        lift_off_plus_state[4] = 0.5 # l distance
        lift_off_plus_state[0+5] = 0.5 # xd, horizontal speed
        lift_off_plus_state[1+5] = 0. # z, vertical speed

        self.apex_z_and_xd_based_off_liftoff_plus(lift_off_plus_state)

    def test_apedx_z_and_xd_based_off_liftoff_plus_2(self):
        lift_off_plus_state = np.zeros(10)
        lift_off_plus_state[1] = 1.51 # foot just touching the ground
        lift_off_plus_state[4] = 0.5 # l distance
        lift_off_plus_state[0+5] = 0.5 # xd, horizontal speed
        lift_off_plus_state[1+5] = 5.0 # z, vertical speed

        self.apex_z_and_xd_based_off_liftoff_plus(lift_off_plus_state)

    def apex_z_and_xd_based_off_liftoff_plus(self, lift_off_plus_state):
        # Use Simulate2dHopper to simulate
        hopper, controller, state_log = Simulate2dHopper(x0 = lift_off_plus_state,
                               duration=2,
                               desired_lateral_velocity = 0.5)

        # Find first apex in simulation
        first_apex_index = self.find_first_apex_in_simulation(state_log)

        # Get simulated max height (z) & horizontal speed (xd) at first apex
        simulated_max_z = self.find_simulated_max_z(state_log, first_apex_index)
        simulated_max_xd = self.find_simulated_max_xd(state_log, first_apex_index)

        # Use Hopper2dController to get max height (z) & speed (xd) based on liftoff state
        calculated_max_z = controller.calculate_apex_z_based_off_liftoff_plus(lift_off_plus_state)
        calculated_max_xd = controller.calculate_apex_xd_based_off_liftoff_plus(lift_off_plus_state)

        # Compare both values
        self.print_and_assert_almost_equal_simulated_and_calculated(
            simulated_max_z, calculated_max_z, 'max height (z)'
        )
        self.print_and_assert_almost_equal_simulated_and_calculated(
            simulated_max_xd, calculated_max_xd, 'max horizontal speed (xd)'
        )

    def print_and_assert_almost_equal_simulated_and_calculated(self, simulated, calculated, name):
        print('Simulated vs calculated ' + name +': ' + '%.2f' % simulated + \
            ' VS ' + '%.2f' % calculated)
        self.assertAlmostEqual(simulated, calculated, 2)


    def find_simulated_max_z(self, state_log, first_apex_index):
        simulated_zs = state_log.data()[1, :]
        simulated_max_z = simulated_zs[first_apex_index]
        return simulated_max_z

    def find_simulated_max_xd(self, state_log, first_apex_index):
        simulated_xds = state_log.data()[0+5, :]
        simulated_max_xd = simulated_xds[first_apex_index]
        return simulated_max_xd

    def find_first_apex_in_simulation(self, state_log):
        simulated_zs = state_log.data()[1, :]
        index = 1 # Skip first time step because simulation is strange
        first_apex_index = -1

        while index < len(simulated_zs) and first_apex_index == -1:
            index = index + 1
            if  simulated_zs[index] > simulated_zs[index - 1] and \
                simulated_zs[index] > simulated_zs[index + 1]:
                first_apex_index = index

        return first_apex_index

    def test_set_liftoff_plus_knowing_desired_height_and_speed(self):
        self.assertEqual('foo'.upper(), 'FOO')
        # Set desired height & speed
        # Use Hopper2dController to calculate liftoff plus state
        # Simulate
        # Get max height & speed

if __name__ == '__main__':
    unittest.main(verbosity=2)
