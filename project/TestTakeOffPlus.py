import unittest
import math
import numpy as np
import inspect
from hopper_2d import Simulate2dHopper


class TestTakeOffPlus(unittest.TestCase):
    # def test_apedx_z_and_xd_based_off_liftoff_plus_1(self):
    #     lift_off_plus_state = np.zeros(10)
    #     lift_off_plus_state[1] = 1.52  # foot just touching the ground
    #     lift_off_plus_state[4] = 0.5  # l distance
    #     lift_off_plus_state[0+5] = 0.5  # xd, horizontal speed
    #     lift_off_plus_state[1+5] = 0.1  # z, vertical speed

    #     self.apex_z_and_xd_based_off_liftoff_plus(lift_off_plus_state)

    # def test_apedx_z_and_xd_based_off_liftoff_plus_2(self):
    #     lift_off_plus_state = np.zeros(10)
    #     lift_off_plus_state[1] = 1.52  # foot just touching the ground
    #     lift_off_plus_state[4] = 0.5  # l distance
    #     lift_off_plus_state[0+5] = 0.5  # xd, horizontal speed
    #     lift_off_plus_state[1+5] = 5.0  # z, vertical speed

    #     self.apex_z_and_xd_based_off_liftoff_plus(lift_off_plus_state)

    # def test_potential_energy(self):
    #     state_1 = np.zeros(10)
    #     state_1[1] = 4.000  # height
    #     state_1[4] = 0.5  # leg length

    #     state_2 = np.zeros(10)
    #     state_2[1] = 2.0000  # height
    #     state_2[4] = 0.5  # leg length

    #     # Use Simulate2dHopper to simulate
    #     hopper, controller, state_log, animation = Simulate2dHopper(x0=state_1,
    #                                                                 duration=0.0,
    #                                                                 actuators_off=True)

    #     # Find calculated energy at apex
    #     calculated_apex_energy_1 = controller.calculate_total_energy(state_1)
    #     calculated_apex_energy_2 = controller.calculate_total_energy(state_2)

    #     # Find simulated energy at apex
    #     plant_context = hopper.CreateDefaultContext()
    #     plant_context.get_mutable_discrete_state_vector().SetFromVector(state_1)
    #     simulated_apex_energy_1 = hopper.CalcPotentialEnergy(plant_context)
    #     print('Simulated energy apex 1:\t' + str(simulated_apex_energy_1))
    #     print('Calculated energy apex 1:\t' + str(calculated_apex_energy_1))

    #     plant_context = hopper.CreateDefaultContext()
    #     plant_context.get_mutable_discrete_state_vector().SetFromVector(state_2)
    #     simulated_apex_energy_2 = hopper.CalcPotentialEnergy(plant_context)
    #     print('Simulated energy apex 2:\t' + str(simulated_apex_energy_2))
    #     print('Calculated energy apex 2:\t' + str(calculated_apex_energy_2))

    #     change_in_simulated = simulated_apex_energy_1 - simulated_apex_energy_2
    #     change_in_calculated = calculated_apex_energy_1 - calculated_apex_energy_2

    #     # Compare both values
    #     self.print_and_assert_almost_equal_simulated_and_calculated(
    #         change_in_simulated, change_in_calculated,
    #         'potential energy in 2 meters',
    #         0
    #     )

    # def test_energy_loss_by_touch_down(self):
    #     apex_state = np.zeros(10)
    #     apex_state[1] = 3.5  # height
    #     apex_state[4] = 0.5  # leg length

    #     # Use Simulate2dHopper to simulate
    #     hopper, controller, state_log, animation = Simulate2dHopper(x0=apex_state,
    #                                                                 duration=2,
    #                                                                 actuators_off=True)

    #     # Find simulated energy at apex
    #     apex_energy = controller.calculate_total_energy(apex_state)

    #     # Find first bottom in simulation
    #     first_bottom_index = self.find_first_bottom_in_simulation(state_log)

    #     # Find simulated energy at bottom
    #     bottom_state = state_log.data()[:, first_bottom_index]
    #     bottom_energy = controller.calculate_total_energy(bottom_state)
    #     simulated_energy_loss = apex_energy - bottom_energy

    #     # Calculate theorical energy loss
    #     calculated_energy_loss = controller.calculate_energy_loss_by_touch_down(
    #         apex_state)

    #     print('\nEnergy apex:\t' + str(apex_energy))
    #     print('Energy bottom:\t' + str(bottom_energy))
    #     print('Energy loss simulation:\t' + str(simulated_energy_loss))
    #     print('Energy loss calculation:\t' + str(calculated_energy_loss))
    #     # Compare both values
    #     self.print_and_assert_almost_equal_simulated_and_calculated(
    #         simulated_energy_loss, calculated_energy_loss, 'touch down energy loss', 0
    #     )

    # def test_energy_loss_by_stance_phase(self):
    #     apex_state = np.zeros(10)
    #     apex_state[1] = 3.5  # height
    #     apex_state[4] = 0.5  # l distance

    #     # Use Simulate2dHopper to simulate
    #     hopper, controller, state_log, animation = Simulate2dHopper(x0=apex_state,
    #                                                                 duration=2,
    #                                                                 actuators_off=True)

    #     # Find first apex in simulation
    #     first_apex_index = self.find_first_apex_in_simulation(state_log)

    #     # Get simulated max height (z) & horizontal speed (xd) at first apex
    #     simulated_max_z = self.find_simulated_max_z(
    #         state_log, first_apex_index)
    #     simulated_max_xd = self.find_simulated_max_xd(
    #         state_log, first_apex_index)

    #     # Calculate max
    #     calculated_energy_loss = controller.calculate_energy_loss_by_stance_phase(
    #         apex_state)
    #     # Transforming all that energy into potential energy
    #     calculated_height_lost = calculated_energy_loss / \
    #         controller.total_mass / controller.gravity
    #     new_height = apex_state[1] - calculated_height_lost
    #     print('Calculated energy loss: ' + str(calculated_energy_loss))
    #     print('Calculated new height: ' + str(new_height))
    #     print('Simulated new height: ' + str(simulated_max_z))

    #     # Compare both values
    #     self.print_and_assert_almost_equal_simulated_and_calculated(
    #         simulated_max_z, new_height, 'max height (z)', 1
    #     )
    #     self.print_and_assert_almost_equal_simulated_and_calculated(
    #         simulated_max_xd, 0.0, 'max horizontal speed (xd)', 1
    #     )

    # def test_energy_loss_by_stance_phase_1(self):
    #     apex_state = np.zeros(10)
    #     apex_state[1] = 3.5  # height
    #     apex_state[4] = 0.5  # l distance
    #     apex_state[0+5] = 0.25  # xd

    #     # Use Simulate2dHopper to simulate
    #     hopper, controller, state_log, animation = Simulate2dHopper(x0=apex_state,
    #                                                                 duration=2,
    #                                                                 desired_lateral_velocity=0.0)

    #     # Find first apex in simulation
    #     first_apex_index = self.find_first_apex_in_simulation(state_log)

    #     # Get simulated max height (z) & horizontal speed (xd) at first apex
    #     simulated_max_z = self.find_simulated_max_z(
    #         state_log, first_apex_index)
    #     simulated_max_xd = self.find_simulated_max_xd(
    #         state_log, first_apex_index)

    #     # Calculate max
    #     calculated_energy_loss = controller.calculate_energy_loss_by_stance_phase(
    #         apex_state)
    #     # Transforming all that energy into potential energy
    #     calculated_height_lost = calculated_energy_loss / \
    #         controller.total_mass / controller.gravity
    #     new_height = apex_state[1] - calculated_height_lost
    #     print('Calculated energy loss: ' + str(calculated_energy_loss))
    #     print('Calculated new height: ' + str(new_height))
    #     print('Simulated new height: ' + str(simulated_max_z))

    #     # Compare both values
    #     self.print_and_assert_almost_equal_simulated_and_calculated(
    #         simulated_max_z, new_height, 'max height (z)', 1
    #     )
    #     self.print_and_assert_almost_equal_simulated_and_calculated(
    #         simulated_max_xd, 0.0, 'max horizontal speed (xd)', 1
    #     )

    # def test_liftoff_beta_calculation(self):
    #     apex_state = np.zeros(10)
    #     apex_state[1] = 3.5  # height
    #     apex_state[4] = 0.5  # l distance
    #     desired_liftoff_beta = 0.0

    #     # Use Simulate2dHopper to simulate
    #     hopper, controller, state_log, animation = Simulate2dHopper(x0=apex_state,
    #                                                                 duration=2,
    #                                                                 actuators_off=True)

    #     touchdown_beta = controller.get_touchdown_beta_for_liftoff_beta(
    #         apex_state, 0.0)

    #     self.assertAlmostEqual(touchdown_beta, desired_liftoff_beta, 1)

    # def test_liftoff_beta_calculation_1(self):
    #     apex_state = np.zeros(10)
    #     apex_state[1] = 3.5  # height
    #     apex_state[4] = 0.5  # l distance
    #     desired_liftoff_beta = 0.25

    #     self.liftoff_beta_calculation(apex_state, desired_liftoff_beta)

    # def test_liftoff_beta_calculation_2(self):
    #     apex_state = np.zeros(10)
    #     apex_state[1] = 3.5  # height
    #     apex_state[2] = 0.5  # theta
    #     apex_state[4] = 0.5  # l distance
    #     desired_liftoff_beta = 0.25

    #     self.liftoff_beta_calculation(apex_state, desired_liftoff_beta)

    # def test_liftoff_beta_calculation_3(self):
    #     apex_state = np.zeros(10)
    #     apex_state[1] = 1.51  # height
    #     apex_state[4] = 0.5  # l distance
    #     apex_state[1+5] = -6.0  # zd
    #     apex_state[2+5] = -0.5  # thetad
    #     desired_liftoff_beta = 0.25

    #     self.liftoff_beta_calculation(apex_state, desired_liftoff_beta)

    # def liftoff_beta_calculation(self, initial_state, desired_liftoff_beta):
    #     # Use Simulate2dHopper to simulate
    #     hopper, controller, state_log, animation = Simulate2dHopper(x0=initial_state,
    #                                                                 duration=2,
    #                                                                 actuators_off=True)

    #     calculated_touchdown_beta = controller.get_touchdown_beta_for_liftoff_beta(
    #         initial_state, desired_liftoff_beta)

    #     print('Calculated desired touchdown beta: ' +
    #           '{:.{}f}'.format(calculated_touchdown_beta, 2))

    #     # Change apex state to have calculated_touchdown_beta
    #     new_alpha = calculated_touchdown_beta - initial_state[2]
    #     initial_state[3] = new_alpha

    #     # Simulate
    #     hopper, controller, state_log, animation = Simulate2dHopper(x0=initial_state,
    #                                                                 duration=2,
    #                                                                 actuators_off=True)

    #     # Find lo beta
    #     simulated_state_index_at_liftoff_minus = self.find_simulated_state_index_at_liftoff_minus(
    #         state_log, controller)

    #     simulated_state_at_liftoff_minus = state_log.data(
    #     )[:, simulated_state_index_at_liftoff_minus]
    #     print('NEW SIMUALTED STATE')
    #     print(simulated_state_at_liftoff_minus)

    #     print('desired_liftoff_beta')
    #     print(desired_liftoff_beta)

    #     simulate_beta_at_liftoff_minus = controller.get_beta(
    #         simulated_state_at_liftoff_minus[2], simulated_state_at_liftoff_minus[3])

    #     print('simulate_beta_at_liftoff_minus')
    #     print(simulate_beta_at_liftoff_minus)

    #     self.assertAlmostEqual(
    #         desired_liftoff_beta, simulate_beta_at_liftoff_minus, 1)

    # def test_liftoff_beta_calculation_2(self):
    #     apex_state = np.zeros(10)
    #     apex_state[1] = 3.5  # height
    #     apex_state[4] = 0.5  # l distance
    #     apex_state[0+5] = 0.5  # xd
    #     desired_liftoff_beta = 0.25

    #     # Use Simulate2dHopper to simulate
    #     hopper, controller, state_log, animation = Simulate2dHopper(x0=apex_state,
    #                                                                 duration=2,
    #                                                                 actuators_off=True)

    #     calculated_touchdown_beta = controller.get_touchdown_beta_for_liftoff_beta(
    #         apex_state, desired_liftoff_beta)

    #     print('Calculated beta: ' +
    #           '{:.{}f}'.format(calculated_touchdown_beta, 2))

    #     # Change apex state to have calculated_touchdown_beta
    #     new_alpha = calculated_touchdown_beta - apex_state[2]
    #     apex_state[3] = new_alpha

    #     # Simulate
    #     hopper, controller, state_log, animation = Simulate2dHopper(x0=apex_state,
    #                                                                 duration=2,
    #                                                                 actuators_off=True)

    #     # Find lo beta
    #     simulated_state_index_at_liftoff_minus = self.find_simulated_state_index_at_liftoff_minus(
    #         state_log, controller)

    #     simulated_state_at_liftoff_minus = state_log.data(
    #     )[:, simulated_state_index_at_liftoff_minus]
    #     print('NEW SIMUALTED STATE')
    #     print(simulated_state_at_liftoff_minus)

    #     print('desired_liftoff_beta')
    #     print(desired_liftoff_beta)

    #     simulate_beta_at_liftoff_minus = controller.get_beta(
    #         simulated_state_at_liftoff_minus[2], simulated_state_at_liftoff_minus[3])

    #     print('simulate_beta_at_liftoff_minus')
    #     print(simulate_beta_at_liftoff_minus)

    #     self.assertAlmostEqual(
    #         desired_liftoff_beta, simulate_beta_at_liftoff_minus, 1)

    # def test_lifoff_minus_state_based_on_touchdown_plus_state(self):
    #     apex_state = np.zeros(10)
    #     apex_state[1] = 3.5  # height
    #     apex_state[4] = 0.5  # l distance

    #     # Use Simulate2dHopper to simulate
    #     hopper, controller, state_log, animation = Simulate2dHopper(x0=apex_state,
    #                                                                 duration=2,
    #                                                                 actuators_off=True)

    #     # Get simulated touchdown plus state
    #     simulated_state_index_at_touchdown_plus = self.find_simulated_state_index_at_touchdown_plus(
    #         state_log, controller)
    #     simulated_state_at_touchdown_plus = state_log.data(
    #     )[:, simulated_state_index_at_touchdown_plus]

    #     print('simulated_state_at_touchdown_plus')
    #     print(simulated_state_at_touchdown_plus)

    #     # Get calcualted touchdown minus state
    #     calculated_state_index_at_liftoff_minus = controller.get_liftoff_minus_state_based_on_touchdown_plus_state(
    #         simulated_state_at_touchdown_plus)

    #     print('calculated_state_index_at_liftoff_minus')
    #     print(calculated_state_index_at_liftoff_minus)

    #     # Get simulated liftoff minus state
    #     simulated_state_index_at_liftoff_minus = self.find_simulated_state_index_at_liftoff_minus(
    #         state_log, controller)
    #     # Take a few timesteps before because liftoff is not precise
    #     simulated_state_at_liftoff_minus = state_log.data(
    #     )[:, simulated_state_index_at_liftoff_minus - 7]

    #     print('simulated_state_at_liftoff_minus')
    #     print(simulated_state_at_liftoff_minus)

    #     # Compare both values
    #     for i in range(calculated_state_index_at_liftoff_minus.shape[0] - 1):
    #         self.print_and_assert_almost_equal_simulated_and_calculated(
    #             simulated_state_at_liftoff_minus[i],
    #             calculated_state_index_at_liftoff_minus[i],
    #             'state at touchdown minus [' + str(i) + ']',
    #             1 if i < 8 else 0
    #         )

    # def test_lifoff_minus_state_based_on_touchdown_plus_state_1(self):
    #     apex_state = np.zeros(10)
    #     apex_state[1] = 1.51  # height
    #     apex_state[4] = 0.5  # l distance
    #     apex_state[1+5] = -6.0  # zd
    #     apex_state[2+5] = -0.5  # thetad

    #     # Use Simulate2dHopper to simulate
    #     hopper, controller, state_log, animation = Simulate2dHopper(x0=apex_state,
    #                                                                 duration=2,
    #                                                                 actuators_off=True)

    #     # Get simulated touchdown plus state
    #     simulated_state_index_at_touchdown_plus = self.find_simulated_state_index_at_touchdown_plus(
    #         state_log, controller)
    #     simulated_state_at_touchdown_plus = state_log.data(
    #     )[:, simulated_state_index_at_touchdown_plus]

    #     print('simulated_state_at_touchdown_plus')
    #     print(simulated_state_at_touchdown_plus)

    #     # Get calcualted touchdown minus state
    #     calculated_state_at_liftoff_minus = controller.get_liftoff_minus_state_based_on_touchdown_plus_state(
    #         simulated_state_at_touchdown_plus)

    #     print('calculated_state_at_liftoff_minus')
    #     print(calculated_state_at_liftoff_minus)

    #     # Get simulated liftoff minus state
    #     simulated_state_index_at_liftoff_minus = self.find_simulated_state_index_at_liftoff_minus(
    #         state_log, controller)
    #     # Take a few timesteps before because liftoff is not precise
    #     simulated_state_at_liftoff_minus = state_log.data(
    #     )[:, simulated_state_index_at_liftoff_minus - 7]

    #     print('simulated_state_at_liftoff_minus')
    #     print(simulated_state_at_liftoff_minus)

    #     simulated_beta = controller.get_beta(
    #         simulated_state_at_liftoff_minus[2], simulated_state_at_liftoff_minus[3])

    #     calculated_beta = controller.get_beta(
    #         calculated_state_at_liftoff_minus[2], calculated_state_at_liftoff_minus[3])

    #     print('simulated_beta')
    #     print(simulated_beta)
    #     print('calculated_beta')
    #     print(calculated_beta)

    #     self.assertAlmostEqual(
    #         calculated_beta, simulated_beta, 1)

    # def test_lifoff_minus_state_based_on_touchdown_plus_state_2(self):
    #     apex_state = np.zeros(10)
    #     apex_state[1] = 1.51  # height
    #     apex_state[3] = -0.1  # alpha
    #     apex_state[4] = 0.5  # l distance
    #     apex_state[0+5] = 1.0  # xd
    #     apex_state[1+5] = -6.0  # zd

    #     # Use Simulate2dHopper to simulate
    #     hopper, controller, state_log, animation = Simulate2dHopper(x0=apex_state,
    #                                                                 duration=2,
    #                                                                 actuators_off=True)

    #     # Get simulated touchdown plus state
    #     simulated_state_index_at_touchdown_plus = self.find_simulated_state_index_at_touchdown_plus(
    #         state_log, controller)
    #     simulated_state_at_touchdown_plus = state_log.data(
    #     )[:, simulated_state_index_at_touchdown_plus]

    #     print('simulated_state_at_touchdown_plus')
    #     print(simulated_state_at_touchdown_plus)

    #     # Get calcualted touchdown minus state
    #     calculated_state_at_liftoff_minus = controller.get_liftoff_minus_state_based_on_touchdown_plus_state(
    #         simulated_state_at_touchdown_plus)

    #     print('calculated_state_at_liftoff_minus')
    #     print(calculated_state_at_liftoff_minus)

    #     # Get simulated liftoff minus state
    #     simulated_state_index_at_liftoff_minus = self.find_simulated_state_index_at_liftoff_minus(
    #         state_log, controller)
    #     # Take a few timesteps before because liftoff is not precise
    #     simulated_state_at_liftoff_minus = state_log.data(
    #     )[:, simulated_state_index_at_liftoff_minus - 7]

    #     simulated_beta = controller.get_beta(
    #         simulated_state_at_liftoff_minus[2], simulated_state_at_liftoff_minus[3])

    #     calculated_beta = controller.get_beta(
    #         calculated_state_at_liftoff_minus[2], calculated_state_at_liftoff_minus[3])

    #     print('simulated_beta')
    #     print(simulated_beta)
    #     print('calculated_beta')
    #     print(calculated_beta)

    #     self.assertAlmostEqual(
    #         calculated_beta, simulated_beta, 1)

    # def test_lifoff_minus_state_based_on_touchdown_plus_state_3(self):
    #     apex_state = np.zeros(10)
    #     apex_state[1] = 3.5  # height
    #     apex_state[3] = 0.1  # alpha
    #     apex_state[4] = 0.5  # l distance

    #     # Use Simulate2dHopper to simulate
    #     hopper, controller, state_log, animation = Simulate2dHopper(x0=apex_state,
    #                                                                 duration=2,
    #                                                                 actuators_off=True)

    #     # Get simulated touchdown plus state
    #     simulated_state_index_at_touchdown_plus = self.find_simulated_state_index_at_touchdown_plus(
    #         state_log, controller)
    #     simulated_state_at_touchdown_plus = state_log.data(
    #     )[:, simulated_state_index_at_touchdown_plus]

    #     print('simulated_state_at_touchdown_plus')
    #     print(simulated_state_at_touchdown_plus)

    #     # Get calcualted touchdown minus state
    #     calculated_state_index_at_liftoff_minus = controller.get_liftoff_minus_state_based_on_touchdown_plus_state(
    #         simulated_state_at_touchdown_plus)

    #     print('calculated_state_index_at_liftoff_minus')
    #     print(calculated_state_index_at_liftoff_minus)

    #     # Get simulated liftoff minus state
    #     simulated_state_index_at_liftoff_minus = self.find_simulated_state_index_at_liftoff_minus(
    #         state_log, controller)
    #     simulated_state_at_liftoff_minus = state_log.data(
    #     )[:, simulated_state_index_at_liftoff_minus]

    #     print('simulated_state_at_liftoff_minus')
    #     print(simulated_state_at_liftoff_minus)

    #     # Compare both values
    #     for i in range(calculated_state_index_at_liftoff_minus.shape[0] - 4):
    #         self.print_and_assert_almost_equal_simulated_and_calculated(
    #             simulated_state_at_liftoff_minus[i],
    #             calculated_state_index_at_liftoff_minus[i],
    #             'state at touchdown minus [' + str(i) + ']',
    #             0
    #         )

    def test_touchdown_minus_state(self):
        apex_state = np.zeros(10)
        apex_state[1] = 3.5  # height
        apex_state[4] = 0.5  # l distance

        self.touchdown_minus_state_based_on_flight_state(apex_state)

    def test_touchdown_minus_state_with_zd(self):
        apex_state = np.zeros(10)
        apex_state[1] = 3.5  # height
        apex_state[4] = 0.5  # l distance
        apex_state[1+5] = 0.5  # zd

        self.touchdown_minus_state_based_on_flight_state(apex_state)

    def test_touchdown_minus_state_with_xd(self):
        apex_state = np.zeros(10)
        apex_state[1] = 3.5  # height
        apex_state[4] = 0.5  # l distance
        apex_state[1+5] = 0.5  # zd
        apex_state[0+5] = 0.5  # xd

        self.touchdown_minus_state_based_on_flight_state(apex_state)

    def test_touchdown_minus_state_with_theta(self):
        apex_state = np.zeros(10)
        apex_state[1] = 3.5  # height
        apex_state[2] = 0.1  # theta
        apex_state[4] = 0.5  # l distance

        self.touchdown_minus_state_based_on_flight_state(apex_state)

    def test_touchdown_minus_state_with_alpha(self):
        apex_state = np.zeros(10)
        apex_state[1] = 3.5  # height
        apex_state[2] = 0.1  # theta
        apex_state[3] = -0.2  # alpha
        apex_state[4] = 0.5  # l distance

        self.touchdown_minus_state_based_on_flight_state(apex_state)

    def test_touchdown_minus_state_with_strong_alpha(self):
        apex_state = np.zeros(10)
        apex_state[1] = 3.5  # height
        apex_state[3] = 0.4  # alpha
        apex_state[4] = 0.5  # l distance

        self.touchdown_minus_state_based_on_flight_state(apex_state)

    def test_touchdown_minus_state_with_all(self):
        apex_state = np.zeros(10)
        apex_state[1] = 3.5  # height
        apex_state[2] = 0.1  # theta
        apex_state[3] = 0.4  # alpha
        apex_state[4] = 0.5  # l distance
        apex_state[0+5] = 0.15  # xd
        apex_state[1+5] = 0.25  # zd

        self.touchdown_minus_state_based_on_flight_state(apex_state)

    def touchdown_minus_state_based_on_flight_state(self, flight_state):
        # Use Simulate2dHopper to simulate
        hopper, controller, state_log, animation = Simulate2dHopper(x0=flight_state,
                                                                    duration=2,
                                                                    actuators_off=True)

        # Get simulated touchdown minus state
        simulated_state_index_at_touchdown_minus = self.find_simulated_state_index_at_touchdown_minus(
            state_log, controller)
        simulated_state_at_touchdown_minus = state_log.data(
        )[:, simulated_state_index_at_touchdown_minus]

        # Get calcualted touchdown minus state
        calculated_state_index_at_touchdown_minus = controller.get_touchdown_minus_state_based_on_flight_state(
            flight_state)

        # Compare both values
        for i in range(calculated_state_index_at_touchdown_minus.shape[0]):
            self.print_and_assert_almost_equal_simulated_and_calculated(
                simulated_state_at_touchdown_minus[i],
                calculated_state_index_at_touchdown_minus[i],
                'state at touchdown minus [' + str(i) + ']',
                1 if i < 8 else 0
            )

    # def test_touchdown_plus_state(self):
    #     apex_state = np.zeros(10)
    #     apex_state[1] = 3.5  # height
    #     apex_state[4] = 0.5  # l distance
    #     apex_state[0+5] = 0.5  # xd
    #     apex_state[1+5] = 0.5  # zd

    #     self.touchdown_plus_state_based_on_flight_state(apex_state)

    # def touchdown_plus_state_based_on_flight_state(self, flight_state):
    #     # Use Simulate2dHopper to simulate
    #     hopper, controller, state_log, animation = Simulate2dHopper(x0=flight_state,
    #                                                                 duration=2,
    #                                                                 actuators_off=True)

    #     # Get simulated touchdown minus state
    #     simulated_state_index_at_touchdown_plus = self.find_simulated_state_index_at_touchdown_plus(
    #         state_log, controller)
    #     simulated_state_at_touchdown_plus = state_log.data(
    #     )[:, simulated_state_index_at_touchdown_plus]

    #     # Get calcualted touchdown minus state
    #     calculated_state_index_at_touchdown_plus = controller.get_touchdown_plus_state_based_on_flight_state(
    #         flight_state)

    #     # Compare both values EXCEPT ld (leg extension derivative)
    #     for i in range(calculated_state_index_at_touchdown_plus.shape[0] - 1):
    #         self.print_and_assert_almost_equal_simulated_and_calculated(
    #             simulated_state_at_touchdown_plus[i],
    #             calculated_state_index_at_touchdown_plus[i],
    #             'state at touchdown minus [' + str(i) + ']',
    #             1 if i < 8 else 0
    #         )

    # def test_foot_position(self):
    #     state = np.zeros(10)
    #     state[1] = 1.5  # height
    #     expected_foot = np.array([0.0, 0.5])

    #     self.foot_position(state, expected_foot)

    # def test_foot_position_extended(self):
    #     state = np.zeros(10)
    #     state[1] = 1.5  # height
    #     state[4] = 0.5  # l distance
    #     expected_foot = np.array([0.0, 0.0])

    #     self.foot_position(state, expected_foot)

    # def test_foot_position_theta(self):
    #     state = np.zeros(10)
    #     state[1] = 1.5      # height
    #     state[2] = 1.57     # theta
    #     state[4] = 0.5      # l distance
    #     expected_foot = np.array([-1.5, state[1]])

    #     self.foot_position(state, expected_foot)

    # def foot_position(self, state, expected_foot_position):
    #     hopper, controller, state_log, animation = Simulate2dHopper(x0=state,
    #                                                                 duration=0.0,
    #                                                                 actuators_off=True)

    #     simulated_foot_position = controller.get_leg_tip_position_from(
    #         state)

    #     self.assertAlmostEqual(
    #         simulated_foot_position[0], expected_foot_position[0], 1)
    #     self.assertAlmostEqual(
    #         simulated_foot_position[1], expected_foot_position[1], 1)

    # def test_get_betas(self):
    #     state = np.zeros(10)
    #     state[1] = 3.0  # height
    #     state[4] = 0.5  # l distance
    #     expected_beta = 0.0

    #     self.get_betas(state, expected_beta)

    # def test_get_betas_with_theta(self):
    #     state = np.zeros(10)
    #     state[1] = 3.0  # height
    #     state[2] = 0.8  # theta
    #     state[4] = 0.5  # l distance
    #     expected_beta = state[2]

    #     self.get_betas(state, expected_beta)

    # def test_get_betas_with_alpha(self):
    #     state = np.zeros(10)
    #     state[1] = 3.0  # height
    #     state[3] = 0.8  # alpha
    #     state[4] = 0.5  # l distance
    #     expected_beta = state[3]

    #     self.get_betas(state, expected_beta)

    # def test_get_betas_with_theta_and_alpha(self):
    #     state = np.zeros(10)
    #     state[1] = 3.0  # height
    #     state[2] = 0.5  # theta
    #     state[3] = 0.4  # alpha
    #     state[4] = 0.5  # l distance
    #     expected_beta = 0.9

    #     self.get_betas(state, expected_beta)

    # def get_betas(self, state, expected_beta):
    #     hopper, controller, state_log, animation = Simulate2dHopper(x0=state,
    #                                                                 duration=0.0,
    #                                                                 actuators_off=True)

    #     beta1 = controller.get_beta(theta=state[2], alpha=state[3])

    #     foot_position = controller.get_leg_tip_position_from(state)
    #     body_position = np.array([state[0], state[1]])
    #     beta2 = controller.get_beta_from(foot_position, body_position)

    #     self.assertAlmostEqual(beta1, beta2, 1)
    #     self.assertAlmostEqual(beta1, expected_beta, 2)

    # def test_get_body_center_of_mass_position(self):
    #     state = np.zeros(10)
    #     state[1] = 3.0  # height
    #     state[4] = 0.5  # l distance
    #     expected_position = np.array([0.0, 3.25])

    #     self.get_body_center_of_mass_position(state, expected_position)

    # def get_body_center_of_mass_position(self, state, expected_position):
    #     hopper, controller, state_log, animation = Simulate2dHopper(x0=state,
    #                                                                 duration=0.0,
    #                                                                 actuators_off=True)

    #     body_position = controller.get_body_position_from(state)

    #     self.assertAlmostEqual(body_position[0], expected_position[0], 2)
    #     self.assertAlmostEqual(body_position[1], expected_position[1], 1)

    # def test_liftoff_minus_state(self):
    #     apex_state = np.zeros(10)
    #     apex_state[1] = 3.5    # height
    #     apex_state[4] = 0.5    # l distance

    #     # Use Simulate2dHopper to simulate
    #     hopper, controller, state_log, animation = Simulate2dHopper(x0=apex_state,
    #                                                                 duration=2,
    #                                                                 actuators_off=True)

    #     # Get simulated liftoff minus state
    #     simulated_state_index_at_liftoff_minus = self.find_simulated_state_index_at_liftoff_minus(
    #         state_log, controller)
    #     simulated_state_at_liftoff_minus = state_log.data(
    #     )[:, simulated_state_index_at_liftoff_minus]

    #     # Get calcualted touchdown minus state
    #     calculated_state_at_liftoff_minus = controller.get_liftoff_minus_state_based_on_flight_state(
    #         apex_state)

    #     self.assertAlmostEqual(
    #         simulated_state_at_liftoff_minus[0],
    #         calculated_state_at_liftoff_minus[0],
    #         2,
    #         'Liftoff x'
    #     )
    #     self.assertAlmostEqual(
    #         simulated_state_at_liftoff_minus[1],
    #         calculated_state_at_liftoff_minus[1],
    #         1,
    #         'Liftoff z'
    #     )
    #     self.assertAlmostEqual(
    #         simulated_state_at_liftoff_minus[0+5],
    #         calculated_state_at_liftoff_minus[0+5],
    #         2,
    #         'Liftoff xd'
    #     )
    #     self.assertAlmostEqual(
    #         simulated_state_at_liftoff_minus[1+5],
    #         calculated_state_at_liftoff_minus[1+5],
    #         0,
    #         'Liftoff zd'
    #     )
    #     self.assertAlmostEqual(
    #         simulated_state_at_liftoff_minus[4],
    #         calculated_state_at_liftoff_minus[4],
    #         1,
    #         'Liftoff l'
    #     )

    # def test_liftoff_minus_state_1(self):
    #     apex_state = np.zeros(10)
    #     apex_state[1] = 3.5  # height
    #     apex_state[3] = -0.1  # alpha
    #     apex_state[4] = 0.5  # l distance
    #     apex_state[0+5] = 1.0  # xd

    #     # Use Simulate2dHopper to simulate
    #     hopper, controller, state_log, animation = Simulate2dHopper(x0=apex_state,
    #                                                                 duration=2,
    #                                                                 actuators_off=True)

    #     # Get simulated liftoff minus state
    #     simulated_state_index_at_liftoff_minus = self.find_simulated_state_index_at_liftoff_minus(
    #         state_log, controller)
    #     simulated_state_at_liftoff_minus = state_log.data(
    #     )[:, simulated_state_index_at_liftoff_minus]

    #     # Get calcualted touchdown minus state
    #     calculated_state_at_liftoff_minus = controller.get_liftoff_minus_state_based_on_flight_state(
    #         apex_state)

    #     print('simulated_state_at_liftoff_minus')
    #     print(simulated_state_at_liftoff_minus)
    #     print('calculated_state_at_liftoff_minus')
    #     print(calculated_state_at_liftoff_minus)

    #     simulated_beta = controller.get_beta(
    #         simulated_state_at_liftoff_minus[2], simulated_state_at_liftoff_minus[3])

    #     calculated_beta = controller.get_beta(
    #         calculated_state_at_liftoff_minus[2], calculated_state_at_liftoff_minus[3])

    #     print('simulated_beta')
    #     print(simulated_beta)
    #     print('calculated_beta')
    #     print(calculated_beta)

    #     self.assertAlmostEqual(
    #         calculated_beta, simulated_beta, 1)

    def test_liftoff_minus_state_2(self):
        apex_state = np.zeros(10)
        apex_state[1] = 3.5  # height
        # theta => The problem is when I add theta, it fails
        apex_state[2] = -0.1
        apex_state[3] = -0.1  # alpha
        apex_state[4] = 0.5  # l distance
        apex_state[0+5] = 1.0  # xd

        # Use Simulate2dHopper to simulate
        hopper, controller, state_log, animation = Simulate2dHopper(x0=apex_state,
                                                                    duration=2,
                                                                    actuators_off=True)

        # Get simulated liftoff minus state
        simulated_state_index_at_liftoff_minus = self.find_simulated_state_index_at_liftoff_minus(
            state_log, controller)
        simulated_state_at_liftoff_minus = state_log.data(
        )[:, simulated_state_index_at_liftoff_minus]

        # Get calcualted touchdown minus state
        calculated_state_at_liftoff_minus = controller.get_liftoff_minus_state_based_on_flight_state(
            apex_state)

        print('simulated_state_at_liftoff_minus')
        print(simulated_state_at_liftoff_minus)
        print('calculated_state_at_liftoff_minus')
        print(calculated_state_at_liftoff_minus)

        simulated_beta = controller.get_beta(
            simulated_state_at_liftoff_minus[2], simulated_state_at_liftoff_minus[3])

        calculated_beta = controller.get_beta(
            calculated_state_at_liftoff_minus[2], calculated_state_at_liftoff_minus[3])

        print('simulated_beta')
        print(simulated_beta)
        print('calculated_beta')
        print(calculated_beta)

        self.assertAlmostEqual(
            calculated_beta, simulated_beta, 1)

    # Should be identical to test_liftoff_minus_state_2, but fails in calculations
    def test_liftoff_minus_state_3(self):
        apex_state = np.zeros(10)
        apex_state[1] = 3.5  # height
        # theta => The problem is when I add theta, it fails
        apex_state[2] = 0.0
        apex_state[3] = -0.2  # alpha
        apex_state[4] = 0.5  # l distance
        apex_state[0+5] = 1.0  # xd

        # Use Simulate2dHopper to simulate
        hopper, controller, state_log, animation = Simulate2dHopper(x0=apex_state,
                                                                    duration=2,
                                                                    actuators_off=True)

        # Get simulated liftoff minus state
        simulated_state_index_at_liftoff_minus = self.find_simulated_state_index_at_liftoff_minus(
            state_log, controller)
        simulated_state_at_liftoff_minus = state_log.data(
        )[:, simulated_state_index_at_liftoff_minus]

        # Get calculated touchdown minus state
        calculated_state_at_liftoff_minus = controller.get_liftoff_minus_state_based_on_flight_state(
            apex_state)

        print('simulated_state_at_liftoff_minus')
        print(simulated_state_at_liftoff_minus)
        print('calculated_state_at_liftoff_minus')
        print(calculated_state_at_liftoff_minus)

        simulated_beta = controller.get_beta(
            simulated_state_at_liftoff_minus[2], simulated_state_at_liftoff_minus[3])

        calculated_beta = controller.get_beta(
            calculated_state_at_liftoff_minus[2], calculated_state_at_liftoff_minus[3])

        print('simulated_beta')
        print(simulated_beta)
        print('calculated_beta')
        print(calculated_beta)

        self.assertAlmostEqual(
            calculated_beta, simulated_beta, 1)

    def test_liftoff_minus_state_with_alpha(self):
        apex_state = np.zeros(10)
        apex_state[1] = 3.5  # height
        apex_state[3] = 0.1  # alpha
        apex_state[4] = 0.5  # l distance

        # Use Simulate2dHopper to simulate
        hopper, controller, state_log, animation = Simulate2dHopper(x0=apex_state,
                                                                    duration=2,
                                                                    actuators_off=True)

        # Get simulated liftoff minus state
        print('STRAING================================================')
        simulated_state_index_at_liftoff_minus = self.find_simulated_state_index_at_liftoff_minus(
            state_log, controller)
        simulated_state_at_liftoff_minus = state_log.data(
        )[:, simulated_state_index_at_liftoff_minus]

        # Get calcualted touchdown minus state
        calculated_state_at_liftoff_minus = controller.get_liftoff_minus_state_based_on_flight_state(
            apex_state)

        print('simulated_state_at_liftoff_minus')
        print(simulated_state_index_at_liftoff_minus)
        print(self.find_simulated_state_index_at_touchdown_plus(
            state_log, controller))
        print(simulated_state_at_liftoff_minus)
        print('calculated_state_at_liftoff_minus')
        print(calculated_state_at_liftoff_minus)

        self.assertAlmostEqual(
            simulated_state_at_liftoff_minus[0],
            calculated_state_at_liftoff_minus[0],
            1,
            'Liftoff x'
        )
        self.assertAlmostEqual(
            simulated_state_at_liftoff_minus[1],
            calculated_state_at_liftoff_minus[1],
            2,
            'Liftoff z'
        )
        self.assertAlmostEqual(
            simulated_state_at_liftoff_minus[0+5],
            calculated_state_at_liftoff_minus[0+5],
            2,
            'Liftoff xd'
        )
        self.assertAlmostEqual(
            simulated_state_at_liftoff_minus[1+5],
            calculated_state_at_liftoff_minus[1+5],
            0,
            'Liftoff zd'
        )
        self.assertAlmostEqual(
            simulated_state_at_liftoff_minus[4],
            calculated_state_at_liftoff_minus[4],
            1,
            'Liftoff l'
        )

    def apex_z_and_xd_based_off_liftoff_plus(self, lift_off_plus_state):
        # Use Simulate2dHopper to simulate
        hopper, controller, state_log, animation = Simulate2dHopper(x0=lift_off_plus_state,
                                                                    duration=2,
                                                                    actuators_off=True)

        # Find first apex in simulation
        first_apex_index = self.find_first_apex_in_simulation(state_log)

        # Get simulated max height (z) & horizontal speed (xd) at first apex
        simulated_max_z = self.find_simulated_max_z(
            state_log, first_apex_index)
        simulated_max_xd = self.find_simulated_max_xd(
            state_log, first_apex_index)

        # Use Hopper2dController to get max height (z) & speed (xd) based on liftoff state
        calculated_max_z = controller.calculate_apex_z_based_off_liftoff_plus(
            lift_off_plus_state)
        calculated_max_xd = controller.calculate_apex_xd_based_off_liftoff_plus(
            lift_off_plus_state)

        # Compare both values
        self.print_and_assert_almost_equal_simulated_and_calculated(
            simulated_max_z, calculated_max_z, 'max height (z)'
        )
        self.print_and_assert_almost_equal_simulated_and_calculated(
            simulated_max_xd, calculated_max_xd, 'max horizontal speed (xd)'
        )

    def print_and_assert_almost_equal_simulated_and_calculated(self, simulated, calculated, name, digits=2):
        print('Simulated vs calculated ' + name + ': ' + '{:.{}f}'.format(simulated, digits + 1) +
              '\t VS \t' + '{:.{}f}'.format(calculated, digits + 1))
        self.assertAlmostEqual(simulated, calculated, digits)

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
        index = 10  # Skip first few time steps
        first_apex_index = -1

        while index < len(simulated_zs) and first_apex_index == -1:
            index = index + 1
            if simulated_zs[index] > simulated_zs[index - 1] and \
                    simulated_zs[index] > simulated_zs[index + 1]:
                first_apex_index = index

        return first_apex_index

    def find_simulated_state_index_at_touchdown_minus(self, state_log, controller):
        index = 0
        touchdown_minus_index = -1
        number_of_states = len(state_log.data()[1, :])

        while touchdown_minus_index == -1 and index < number_of_states:
            index = index + 1
            if controller.is_foot_in_contact(state_log.data()[:, index]):
                touchdown_minus_index = index - 1

        return touchdown_minus_index

    def find_simulated_state_index_at_touchdown_plus(self, state_log, controller):
        index = 0
        touchdown_plus_index = -1
        number_of_states = len(state_log.data()[1, :])

        while touchdown_plus_index == -1 and index < number_of_states:
            index = index + 1
            if controller.is_foot_in_contact(state_log.data()[:, index]):
                touchdown_plus_index = index

        return touchdown_plus_index

    def find_simulated_state_index_at_liftoff_minus(self, state_log, controller):
        # Start at touchdown plus
        index = self.find_simulated_state_index_at_touchdown_plus(
            state_log, controller)
        liftoff_minus_index = -1
        number_of_states = len(state_log.data()[1, :])

        while liftoff_minus_index == -1 and index < number_of_states:
            index = index + 1
            # foot_height = controller.get_leg_tip_position_from(
            #     state_log.data()[:, index])[1]
            # print('CONTACT:\t' + str(not controller.is_foot_in_contact(state_log.data()[:, index])) +
            #       '\t\t' + str(index) + '\t\t' + str(foot_height) + '\t\t leg extension: ' + str(state_log.data()[4, index]))
            if not controller.is_foot_in_contact(state_log.data()[:, index]):
                liftoff_minus_index = index - 1

        return liftoff_minus_index

    def find_first_bottom_in_simulation(self, state_log):
        simulated_zs = state_log.data()[1, :]
        index = 1
        first_bottom_index = -1

        while index < len(simulated_zs) and first_bottom_index == -1:
            index = index + 1
            if simulated_zs[index] < simulated_zs[index - 1] and \
                    simulated_zs[index] < simulated_zs[index + 1]:
                first_bottom_index = index

        return first_bottom_index

    # Do a test with xd and theta is not 0 and do the forward calculations to figure out
    # the location of the next apex

    # Then forward iterator thought the stance phase, assuming join with the floor and point mass body

    # Do a test that makes it maintain it's height
    # Add a flag to turn on/off maintaining desired height in the controller
    # def test_set_liftoff_plus_knowing_desired_height_and_speed(self):
    #     self.assertEqual('foo'.upper(), 'FOO')
    #     # Set desired height & speed
    #     # Use Hopper2dController to calculate liftoff plus state
    #     # Simulate
    #     # Get max height & speed


if __name__ == '__main__':
    unittest.main(verbosity=2)
