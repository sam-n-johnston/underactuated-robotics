import unittest
import numpy as np
import inspect
from hopper_2d import Simulate2dHopper

class TestTakeOffPlus(unittest.TestCase):
    def test_apedx_z_and_xd_based_off_liftoff_plus_1(self):
        lift_off_plus_state = np.zeros(10)
        lift_off_plus_state[1] = 1.52 # foot just touching the ground
        lift_off_plus_state[4] = 0.5 # l distance
        lift_off_plus_state[0+5] = 0.5 # xd, horizontal speed
        lift_off_plus_state[1+5] = 0.1 # z, vertical speed

        self.apex_z_and_xd_based_off_liftoff_plus(lift_off_plus_state)

    def test_apedx_z_and_xd_based_off_liftoff_plus_2(self):
        lift_off_plus_state = np.zeros(10)
        lift_off_plus_state[1] = 1.52 # foot just touching the ground
        lift_off_plus_state[4] = 0.5 # l distance
        lift_off_plus_state[0+5] = 0.5 # xd, horizontal speed
        lift_off_plus_state[1+5] = 5.0 # z, vertical speed

        self.apex_z_and_xd_based_off_liftoff_plus(lift_off_plus_state)

    def test_potential_energy(self):
        state_1 = np.zeros(10)
        state_1[1] = 4.000 # height
        state_1[4] = 0.5 # leg length

        state_2 = np.zeros(10)
        state_2[1] = 2.0000 # height
        state_2[4] = 0.5 # leg length

        state_3 = np.zeros(10)
        state_3[1] = 1.50000 # height
        state_3[4] = 0.5 # leg length

        # Use Simulate2dHopper to simulate
        hopper, controller, state_log, animation = Simulate2dHopper(x0 = state_1,
                               duration=2,
                               desired_lateral_velocity = 0.0)

        # Find calculated energy at apex
        calculated_apex_energy_1 = controller.calculate_total_energy(state_1)
        calculated_apex_energy_2 = controller.calculate_total_energy(state_2)
        calculated_apex_energy_3 = controller.calculate_total_energy(state_3)

        # Find simulated energy at apex
        plant_context = hopper.CreateDefaultContext()
        plant_context.get_mutable_discrete_state_vector().SetFromVector(state_1)
        simulated_apex_energy = hopper.CalcPotentialEnergy(plant_context)
        body = hopper.GetBodyByName('body')
        pose = hopper.EvalBodyPoseInWorld(plant_context, body)
        print('pose state_1')
        print(hopper.CalcCenterOfMassPosition(plant_context))
        print(pose.GetAsMatrix4())
        # print(inspect.getmembers(body))
        # print(body.default_com())
        # print(inspect.getmembers(body.default_spatial_inertia()))
        print(body.default_spatial_inertia().get_mass())
        # print(inspect.getmembers(body.default_unit_inertia()))
        # print(body.default_unit_inertia())
        # body.SetMass(1.0)
        print('Simulated energy apex 1:\t'  + str(simulated_apex_energy))
        print('Calculated energy apex:\t'  + str(calculated_apex_energy_1))

        plant_context = hopper.CreateDefaultContext()
        plant_context.get_mutable_discrete_state_vector().SetFromVector(state_2)
        simulated_apex_energy = hopper.CalcPotentialEnergy(plant_context)
        body = hopper.GetBodyByName('body')
        pose = hopper.EvalBodyPoseInWorld(plant_context, body)
        print('pose state_2')
        print(hopper.CalcCenterOfMassPosition(plant_context))
        print(pose.GetAsMatrix4())
        # print(inspect.getmembers(pose))
        print('Simulated energy apex 2:\t'  + str(simulated_apex_energy))
        print('Calculated energy apex:\t'  + str(calculated_apex_energy_2))

        plant_context = hopper.CreateDefaultContext()
        plant_context.get_mutable_discrete_state_vector().SetFromVector(state_3)
        simulated_apex_energy = hopper.CalcPotentialEnergy(plant_context)
        body = hopper.GetBodyByName('body')
        pose = hopper.EvalBodyPoseInWorld(plant_context, body)
        print('pose state_3')
        print(hopper.CalcCenterOfMassPosition(plant_context))
        print(pose.GetAsMatrix4())
        # print(inspect.getmembers(pose))
        print('Simulated energy apex 3:\t'  + str(simulated_apex_energy))
        print('Calculated energy apex:\t'  + str(calculated_apex_energy_3))

        # Compare both values
        # self.print_and_assert_almost_equal_simulated_and_calculated(
        #     simulated_energy_loss, calculated_energy_loss, 'touch down energy loss'
        # )


    # def test_energy_loss_by_touch_down(self):
    #     apex_state = np.zeros(10)
    #     # TODO: If I change this height, it stops working...
    #     apex_state[1] = 3.5 # height
    #     apex_state[4] = 0.5 # leg length

    #     # Use Simulate2dHopper to simulate
    #     hopper, controller, state_log, animation = Simulate2dHopper(x0 = apex_state,
    #                            duration=2,
    #                            desired_lateral_velocity = 0.0)

    #     # vals = hopper.CalcGravityGeneralizedForces()
    #     # print('vals')
    #     # print(vals)

    #     # Find first bottom in simulation
    #     first_bottom_index = self.find_first_bottom_in_simulation(state_log)

    #     # Find simulated energy at apex
    #     apex_energy = controller.calculate_total_energy(apex_state)

    #     # Find simulated energy at bottom
    #     bottom_state = state_log.data()[:, first_bottom_index]
    #     bottom_energy = controller.calculate_total_energy(bottom_state)
    #     simulated_energy_loss = apex_energy - bottom_energy

    #     # Calculate theorical energy loss
    #     calculated_energy_loss = controller.calculate_energy_loss_by_touch_down(bottom_state)

    #     # print('==========================================================')
    #     # print('Energy2:\t' + str(state_log.data()[:, 2]))
    #     # for i in range (1000):
    #     #     if controller.is_foot_in_contact(state_log.data()[:, i]):
    #     #         print('========================= IN CONTACT =========================')
    #     #     print('Energy' + str(i) + ':\t'  + str(controller.calculate_total_energy(state_log.data()[:, i])))
    #     print('')
    #     print('Energy apex:\t'  + str(apex_energy))
    #     print('apex:\t'  + str(apex_state))
    #     print('Energy bottom:\t'  + str(bottom_energy))
    #     print('bottom:\t'  + str(bottom_state))
    #     # Compare both values
    #     self.print_and_assert_almost_equal_simulated_and_calculated(
    #         simulated_energy_loss, calculated_energy_loss, 'touch down energy loss'
    #     )

    # def test_energy_loss_by_stance_phase(self):
    #     apex_state = np.zeros(10)
    #     apex_state[1] = 2.0 # height
    #     apex_state[4] = 0.5 # l distance

    #     # Use Simulate2dHopper to simulate
    #     hopper, controller, state_log, animation = Simulate2dHopper(x0 = apex_state,
    #                            duration=2,
    #                            desired_lateral_velocity = 0.0)

    #     # Find first apex in simulation
    #     first_apex_index = self.find_first_apex_in_simulation(state_log)

    #     # Get simulated max height (z) & horizontal speed (xd) at first apex
    #     simulated_max_z = self.find_simulated_max_z(state_log, first_apex_index)
    #     simulated_max_xd = self.find_simulated_max_xd(state_log, first_apex_index)

    #     # Calculate max 
    #     energy_lost = controller.calculate_energy_loss_by_stance_phase(apex_state)
    #     # Transforming all that energy into potential energy
    #     g = 9.81
    #     m = controller.m_b + controller.m_f
    #     height_lost = energy_lost / m / g
    #     new_height = apex_state[1] - height_lost
    #     print('Total energy lost: ' + str(energy_lost))
    #     print('New height: ' + str(new_height))

    #     # Compare both values
    #     self.print_and_assert_almost_equal_simulated_and_calculated(
    #         simulated_max_z, new_height, 'max height (z)'
    #     )
    #     self.print_and_assert_almost_equal_simulated_and_calculated(
    #         simulated_max_xd, 0.0, 'max horizontal speed (xd)'
    #     )

    def apex_z_and_xd_based_off_liftoff_plus(self, lift_off_plus_state):
        # Use Simulate2dHopper to simulate
        hopper, controller, state_log, animation = Simulate2dHopper(x0 = lift_off_plus_state,
                               duration=2,
                               desired_lateral_velocity = 0.0)

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
        print('Simulated vs calculated ' + name +': ' + '%.5f' % simulated + \
            ' VS ' + '%.5f' % calculated)
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
        index = 10 # Skip first few time steps
        first_apex_index = -1

        while index < len(simulated_zs) and first_apex_index == -1:
            index = index + 1
            if  simulated_zs[index] > simulated_zs[index - 1] and \
                simulated_zs[index] > simulated_zs[index + 1]:
                first_apex_index = index

        return first_apex_index

    # def find_first_bottom_in_simulation(self, state_log):
    #     simulated_zs = state_log.data()[1, :]
    #     index = 1
    #     first_bottom_index = -1

    #     while index < len(simulated_zs) and first_bottom_index == -1:
    #         index = index + 1
    #         if  simulated_zs[index] < simulated_zs[index - 1] and \
    #             simulated_zs[index] < simulated_zs[index + 1]:
    #             first_bottom_index = index

    #     return first_bottom_index

    # def test_set_liftoff_plus_knowing_desired_height_and_speed(self):
    #     self.assertEqual('foo'.upper(), 'FOO')
    #     # Set desired height & speed
    #     # Use Hopper2dController to calculate liftoff plus state
    #     # Simulate
    #     # Get max height & speed

if __name__ == '__main__':
    unittest.main(verbosity=2)
