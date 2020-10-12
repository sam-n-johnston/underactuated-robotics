# -*- coding: utf8 -*-

import argparse
import math
import os.path
import time
import inspect

import numpy as np

from pydrake.all import (
    MultibodyPlant,
    PlanarSceneGraphVisualizer,
    DiagramBuilder,
    Simulator, VectorSystem,
    ConstantVectorSource,
    SignalLogger,
    AbstractValue,
    Parser,
    PortDataType,
    UniformGravityFieldElement,
    default_model_instance
)
from IPython.display import HTML
import matplotlib.pyplot as plt
from drake import lcmt_viewer_load_robot
from pydrake.common.eigen_geometry import Quaternion
from pydrake.geometry import DispatchLoadMessage, SceneGraph
from pydrake.lcm import DrakeMockLcm
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.systems.rendering import PoseBundle


class Hopper2dController(VectorSystem):
    def __init__(self, hopper,
                 desired_lateral_velocity=0.0,
                 print_period=0.0):
        # hopper = a rigid body tree representing the 1D hopper
        # desired_lateral_velocity = How fast should the controller
        #  aim to run sideways?
        # print_period = print to console to indicate sim progress
        #  if nonzero
        VectorSystem.__init__(self,
                              10,  # 10 inputs: x, z, theta, alpha, l, and their derivatives
                              2)  # 2 outputs: Torque on thigh, and force on the leg extension
        #  link. (The passive spring for on the leg is calculated as
        #  part of this output.)
        self.hopper = hopper
        self.desired_lateral_velocity = desired_lateral_velocity
        self.print_period = print_period
        self.last_print_time = -print_period
        # Remember what the index of the foot is
        self.foot_frame = hopper.GetFrameByName("foot")
        self.body_frame = hopper.GetFrameByName("body")
        self.world_frame = hopper.world_frame()

        # The context for the hopper
        self.plant_context = self.hopper.CreateDefaultContext()

        # Default parameters for the hopper -- should match
        # raibert_hopper_1d.sdf, where applicable.
        # You're welcome to use these, but you probably don't need them.
        self.hopper_leg_length = 2.0
        self.body_size_height = 0.5
        # Based on the potential energy from plant.CalcPotentialEnergy(plant_context)
        self.total_mass = 3.095499
        # Based on the potential energy from plant.CalcPotentialEnergy(plant_context)
        mass_factor = 2.81409
        self.m_f = 0.1  # If I increase this mass, it decreases the energy loss... this doesn't make any sense
        self.m_b = self.total_mass - self.m_f
        self.l_max = 0.5
        self.gravity = 9.81

        # This is an arbitrary choice of spring constant for the leg.
        self.K_l = 100
        self.desired_alpha_array = []

    def calculate_energy_loss_by_stance_phase(self, flight_phase):
        return self.calculate_energy_loss_by_touch_down(flight_phase) + \
            self.calculate_energy_loss_by_lift_off(flight_phase)

    def calculate_kinetic_energy(self, mass, speed):
        return 1.0 / 2.0 * mass * speed ** 2.0

    def calculate_potential_energy(self, mass, height):
        return self.gravity * mass * height

    def spring_potential_energy_with(self, leg_compression_amount):
        return 1.0 / 2.0 * self.K_l * leg_compression_amount ** 2.0

    def spring_potential_energy(self, state):
        # TODO: use correct value
        l_rest = 1.0

        # Passive spring force
        leg_compression_amount = l_rest - state[4]
        return self.spring_potential_energy_with(leg_compression_amount)

    def spring_force(self, leg_compression_amount):
        return self.K_l * leg_compression_amount

    def potential_energy_body(self, state):
        return self.calculate_potential_energy(self.m_b, state[1])

    def potential_energy_foot(self, state):
        # Assuming foot's mass in a point mass in the middle
        # TODO: take into  account for alpha and theta
        # (use the get_foot_position_from or similar)
        is_foot_in_air = state[1] + \
            self.body_size_height > self.hopper_leg_length
        if is_foot_in_air:
            # Assuming mass is in middle of leg
            foot_height = state[1] + self.body_size_height - \
                self.hopper_leg_length / 2.0
        else:
            foot_height = self.hopper_leg_length / 2.0

        return self.calculate_potential_energy(self.m_f, foot_height)

    def calculate_total_energy(self, state):
        kinetic_energy_body = self.calculate_kinetic_energy(self.m_b, state[0+5]) + \
            self.calculate_kinetic_energy(self.m_b, state[1+5])
        kinetic_energy_foot = self.calculate_kinetic_energy(self.m_f, state[0+5]) + \
            self.calculate_kinetic_energy(self.m_f, state[1+5])
        potential_energy_body = self.potential_energy_body(state)
        potential_energy_foot = self.potential_energy_foot(state)
        spring_potential_energy = self.spring_potential_energy(state)

        energy = kinetic_energy_body + kinetic_energy_foot +\
            potential_energy_body + potential_energy_foot + spring_potential_energy

        return energy

    def get_touchdown_minus_state_based_on_flight_state(self, flight_phase):
        total_energy_flight = self.calculate_total_energy(flight_phase)

        # Get touch down minus speeds (before impact)
        xd_flight = flight_phase[0+5]
        xd_flight_energy = self.calculate_kinetic_energy(
            self.m_b + self.m_f, xd_flight)

        # Potential energy
        # TODO: take into  account for alpha and theta
        # (use the get_foot_position_from or similar)
        body_z_touchdown_minus = self.hopper_leg_length - self.body_size_height
        # TODO: take into  account for alpha and theta
        # (use the get_foot_position_from or similar)
        foot_z_touchdown_minus = self.hopper_leg_length / 2.0

        potential_energy_foot = self.calculate_potential_energy(
            self.m_f, foot_z_touchdown_minus)
        potential_energy_body = self.calculate_potential_energy(
            self.m_b, body_z_touchdown_minus)

        # Spring energy
        leg_compression_amount_minus = 0.5
        spring_potential_energy = self.spring_potential_energy_with(
            leg_compression_amount_minus)

        # Kinectic energy
        zd_energy = total_energy_flight - xd_flight_energy - \
            potential_energy_foot - potential_energy_body - spring_potential_energy
        zd_minus = -math.sqrt(2.0 * zd_energy / (self.m_b + self.m_f))

        # Get time estimation
        zd_current = flight_phase[1+5]
        time_elapsed = (zd_current - zd_minus) / self.gravity

        touchdown_minus_state = np.zeros(10)
        touchdown_minus_state[0] = time_elapsed * xd_flight
        touchdown_minus_state[1] = body_z_touchdown_minus
        touchdown_minus_state[4] = leg_compression_amount_minus
        touchdown_minus_state[0+5] = xd_flight
        touchdown_minus_state[1+5] = zd_minus

        return touchdown_minus_state

    def get_touchdown_plus_state_based_on_flight_state(self, flight_phase):
        return self.get_touchdown_minus_state_based_on_flight_state(flight_phase)

    def get_beta_from(self, foot_position, body_position):
        x_diff = body_position[0] - foot_position[0]
        z_diff = body_position[1] - foot_position[1]
        return math.atan2(x_diff, z_diff)

    def get_beta(self, theta, alpha):
        return theta + alpha

    def get_beta_from_touchdown_state(self, touchdown_state):
        return self.get_beta(theta=touchdown_state[2], alpha=touchdown_state[3])

    def get_foot_position_from(self, state):
        context = self.hopper.CreateDefaultContext()
        context.get_mutable_discrete_state_vector().SetFromVector(state)
        # Run out the forward kinematics of the robot
        # to figure out where the foot is in world frame.
        foot_point = np.array([0.0, 0.0, -self.hopper_leg_length / 2.0])
        foot_point_in_world = self.hopper.CalcPointsPositions(
            context,
            self.foot_frame,
            foot_point,
            self.world_frame
        )
        return np.array([foot_point_in_world[0, 0], foot_point_in_world[2, 0]])

    def get_body_position_from(self, state):
        context = self.hopper.CreateDefaultContext()
        context.get_mutable_discrete_state_vector().SetFromVector(state)
        # Run out the forward kinematics of the robot
        # to figure out where the body is in world frame.
        # body_point = np.array([0.0, 0.0, 0.0])
        body_point = np.array([0.0, 0.0, self.body_size_height / 2.0])
        body_point_in_world = self.hopper.CalcPointsPositions(
            context,
            self.body_frame,
            body_point,
            self.world_frame
        )
        return np.array([body_point_in_world[0, 0], body_point_in_world[2, 0]])

    '''
        Calcualtes the leg length from the tip of the foot
        to the center of the body
    '''

    def get_leg_length(self, foot_position, body_position):
        x_diff = foot_position[0] - body_position[0]
        z_diff = foot_position[1] - body_position[1]
        distance = math.sqrt(x_diff ** 2.0 + z_diff ** 2.0)
        return distance

    def get_liftoff_minus_state_based_on_flight_state(self, flight_phase):
        touchdown_minus_state = self.get_touchdown_minus_state_based_on_flight_state(
            flight_phase)
        timestep = 0.0005
        current_time = 0.0
        current_state = np.copy(touchdown_minus_state)
        foot_position = self.get_foot_position_from(
            touchdown_minus_state)
        found_liftoff_minus_state = False
        f_gravity_body = self.m_b * self.gravity
        f_gravity_foot = self.m_f * self.gravity
        beta = self.get_beta(
            theta=current_state[2], alpha=current_state[3])

        while not found_liftoff_minus_state and current_time < 2.0:
            l_rest = 1.0
            spring_force = self.spring_force(l_rest - current_state[4])

            # Spring is pushing back only the body, not the foot's mass
            f_gravity_along_leg_frame = f_gravity_body * math.cos(beta)
            leg_force = spring_force - f_gravity_along_leg_frame

            acceleration_along_leg_frame = leg_force / self.m_b

            # Calculate rotational acceleration
            body_position = self.get_body_position_from(
                current_state)
            # Maybe the leg length is not correct <= TODO: Let's test this, I think it's the culprit.
            leg_length = self.get_leg_length(foot_position, body_position)

            # f_gravity_body is good, and f_gravity_body_perpendicular_to_leg_frame is good
            f_gravity_body_perpendicular_to_leg_frame = f_gravity_body * \
                math.sin(beta)
            # f_gravity_foot_perpendicular_to_leg_frame is good
            f_gravity_foot_perpendicular_to_leg_frame = f_gravity_foot * \
                math.sin(beta)

            # Maybe the center of mass of the leg isn't in the middle? I doubt it
            f_gravity_foot_torque = f_gravity_foot_perpendicular_to_leg_frame * \
                self.hopper_leg_length / 2.0
            f_gravity_body_torque = f_gravity_body_perpendicular_to_leg_frame * leg_length

            f_gravity_torque = f_gravity_body_torque + f_gravity_foot_torque

            # Calculate moment of inertia
            moment_of_inertia = self.m_f * \
                (self.hopper_leg_length / 2.0) ** 2.0 + \
                self.m_b * (leg_length) ** 2.0

            # Calculate rotational acceleration
            rotational_acceleration = f_gravity_torque / moment_of_inertia

            # Transform to linear acceleration
            acceleration_perpendicular_to_leg_frame = rotational_acceleration * leg_length

            #  Calculate new acceleration in x & z frames (with all forces)
            #  x_acceleration_along_leg_frame should be good since z_acceleration_along_leg_frame is good
            x_acceleration_along_leg_frame = acceleration_along_leg_frame * \
                math.sin(beta)
            z_acceleration_along_leg_frame = acceleration_along_leg_frame * \
                math.cos(beta)
            # This calculation seems fine, so acceleration_perpendicular_to_leg_frame might be the problem
            x_acceleration_perpendicular_to_leg_frame = acceleration_perpendicular_to_leg_frame * \
                math.cos(beta)
            z_acceleration_perpendicular_to_leg_frame = acceleration_perpendicular_to_leg_frame * \
                math.sin(beta)

            # TODO: Something in x_acceleration isn't correct...
            x_acceleration = x_acceleration_along_leg_frame + \
                x_acceleration_perpendicular_to_leg_frame
            z_acceleration = z_acceleration_along_leg_frame + \
                z_acceleration_perpendicular_to_leg_frame

            # Set new speeds
            # x_acceleration should slow down when it lands and then re-accelerate
            current_state[0+5] = current_state[0+5] + x_acceleration * timestep
            current_state[1+5] = current_state[1+5] + z_acceleration * timestep

            # Set new positions
            current_state[0] = current_state[0] + current_state[0+5] * timestep
            current_state[1] = current_state[1] + current_state[1+5] * timestep

            # TODO: Fix this to send in the position of the center of the body
            body_position = self.get_body_position_from(current_state)
            leg_length = self.get_leg_length(foot_position, body_position)

            current_state[4] = leg_length - self.hopper_leg_length / \
                2.0 - self.body_size_height / 2.0

            # Get beta based on
            beta = self.get_beta_from(foot_position, body_position)

            current_time = current_time + timestep

            if current_state[4] >= 0.5:
                print('\nIn the air now!')
                print(current_time)
                print(beta)
                found_liftoff_minus_state = True

        if not found_liftoff_minus_state:
            raise Exception('The robot never left the ground')

        return current_state

    def calculate_energy_loss_by_touch_down(self, flight_phase):
        touchdown_minus_state = self.get_touchdown_minus_state_based_on_flight_state(
            flight_phase)

        xd_minus = touchdown_minus_state[0+5]
        zd_minus = touchdown_minus_state[1+5]

        kinetic_energy_lost_in_foot = self.calculate_kinetic_energy(self.m_f, xd_minus) + \
            self.calculate_kinetic_energy(self.m_f, zd_minus)

        return kinetic_energy_lost_in_foot

    def controller(self):
        # 1. For the desired height & speed, calculate liftoff angle =>
        #      Check if this assumption is correct. I'm assuming that liftoff angle
        #      will be the same as the liftoff speed vector of the robot. I can verify
        #      at least part of this with another test where the robot has an initial leg
        #      position and horizontal speed.
        # 2. Estimate stance energy loss and set l_rest actuation
        #      Check if I need to refine the energy loss if the touchdown and liftoff
        #      are not with the leg straight down
        # 3. Find the correct leg angle for touchdown
        #      Calculate forward in time to see if the liftoff angle will be correct
        # 4. Repeat steps 2 and 3 until convergence (since one affects the other)
        return

    def calculate_energy_loss_by_lift_off(self, flight_phase):
        # Get total energy minus (before impact)
        total_energy_minus = self.calculate_total_energy(flight_phase) - \
            self.calculate_energy_loss_by_touch_down(flight_phase)

        # Get lift off minus speeds (before impact)
        xd_minus = flight_phase[0+5]
        xd_minus_energy = self.calculate_kinetic_energy(self.m_b, xd_minus)
        # These are assuming that the leg is straight down.
        potential_energy_foot = self.calculate_potential_energy(
            self.m_f, self.hopper_leg_length / 2.0)
        potential_energy_body = self.calculate_potential_energy(
            self.m_b, self.hopper_leg_length - self.body_size_height)
        # TODO: use correct value
        l_rest = 1.0
        # Passive spring force
        sprint_potential_energy = 1.0 / 2.0 * self.K_l * (l_rest - 0.5) ** 2.0

        zd_energy = total_energy_minus - xd_minus_energy - \
            potential_energy_foot - potential_energy_body - sprint_potential_energy
        zd_minus = math.sqrt(2 * zd_energy / self.m_b)

        # Calculate body speeds plus (after impact)
        xd_plus = self.m_b / (self.m_b + self.m_f) * xd_minus
        zd_plus = self.m_b / (self.m_b + self.m_f) * zd_minus

        # Calculate energy plus (after impact)
        total_energy_plus = self.calculate_kinetic_energy(self.m_b + self.m_f, xd_plus) + \
            self.calculate_kinetic_energy(self.m_b + self.m_f, zd_plus) + \
            potential_energy_foot + potential_energy_body + sprint_potential_energy

        # Calculate energy loss
        return total_energy_minus - total_energy_plus

    def calculate_apex_xd_based_off_liftoff_plus(self, lift_off_plus_state):
        return lift_off_plus_state[0+5]

    def is_foot_in_contact(self, state):
        foot_point_in_world = self.get_foot_position_from(state)
        # print('foot_point_in_world')
        # print(foot_point_in_world)
        # and state[4] < self.l_max
        in_contact = foot_point_in_world[1] <= 0.01

        return in_contact

    def calculate_apex_z_based_off_liftoff_plus(self, lift_off_plus_state):
        lift_off_zd = lift_off_plus_state[1+5]
        apex_z = lift_off_zd ** 2 / (2 * self.gravity)
        return apex_z + lift_off_plus_state[1]

    def calculate_moment_of_inertia(self):
        return self.m_f * self.l_max ** 2.

    def calculate_lift_off_angle(self):
        return math.atan2(self.desired_lateral_velocity, (2 * self.gravity * self.desired_height) ** (1. / 2.))

    def calculate_time_required_btwn_lo_and_td(self):
        lift_off_height = math.cos(
            self.calculate_lift_off_angle()) * self.hopper_leg_length
        intial_vertical_speed = - \
            (2 * self.gravity * self.desired_height) ** (1. / 2.)
        final_vertical_speed = -intial_vertical_speed
        return (final_vertical_speed - intial_vertical_speed) / self.gravity

    def calculate_desired_acceleration(self):
        current_position = -self.calculate_lift_off_angle()
        desired_position = 0
        t = self.calculate_time_required_btwn_lo_and_td() / 2.
        initial_alpha_d = 0
        return 2 * (desired_position - current_position - initial_alpha_d * t) / (t ** 2.)

    def get_x_err(self, X):
        x, z, theta, alpha, l = X[0:5]
        xd, zd, thetad, alphad, ld = X[5:10]
        xd_desired = self.desired_lateral_velocity
        theta_desired = 0
        K1 = 1.2
        K2 = 4.
        K3 = 2.

        return K1 * (xd - xd_desired) + K2 * (theta - theta_desired) + K3 * thetad

    def get_desired_alpha_in_air(self, X):
        x, z, theta, alpha, l = X[0:5]
        xd, zd, thetad, alphad, ld = X[5:10]
        mb = self.m_b
        mf = self.m_f
        l = self.hopper_leg_length
        K_err = 0.1

        x_err = K_err * self.get_x_err(X)
        opposing_side = ((mb + mf) * x_err) / (l * mb)

        if opposing_side > 1.:
            opposing_side = 1.

        if opposing_side < -1.:
            opposing_side = -1.

        desired_alpha = - math.asin(opposing_side) - theta
        self.desired_alpha_array.append(desired_alpha)
        return desired_alpha

    def get_desired_alpha_in_contact(self, X):
        x, z, theta, alpha, l = X[0:5]

        return alpha + theta

    def PD_controller(self, X, in_contact, in_air):
        x, z, theta, alpha, l = X[0:5]
        xd, zd, thetad, alphad, ld = X[5:10]

        if in_contact:
            Kp = -20.
            Kv = -5.
            alpha_desired = self.get_desired_alpha_in_contact(X)
        elif in_air:
            Kp = -20.
            Kv = -5.
            alpha_desired = self.get_desired_alpha_in_air(X)
        else:
            return 0.

        return Kp * (alpha - alpha_desired) + Kv * (alphad)

    def ChooseThighTorque(self, X):
        ''' Given the system state X,
            returns a (scalar) leg angle torque to exert. '''
        x, z, theta, alpha, l = X[0:5]
        xd, zd, thetad, alphad, ld = X[5:10]

        # Run out the forward kinematics of the robot
        # to figure out where the foot is in world frame.
        foot_point = np.array([0.0, 0.0, -self.hopper_leg_length])
        foot_point_in_world = self.hopper.CalcPointsPositions(self.plant_context,
                                                              self.foot_frame, foot_point, self.world_frame)
        in_contact = foot_point_in_world[2] <= 0.01
        in_air = foot_point_in_world[2] >= 0.1

        # It's all yours from here.
        # Implement a controller that:
        #  - Controls xd to self.desired_lateral_velocity
        #  - Attempts to keep the body steady (theta = 0)

        torque = self.PD_controller(X, in_contact, in_air)

        return 0.0  # torque

    def ChooseSpringRestLength(self, X):
        ''' Given the system state X,
            returns a (scalar) rest length of the leg spring.
            We can command this instantaneously, as
            the actual system being simulated has perfect
            force control of its leg extension. '''
        # Unpack states
        x, z, theta, alpha, l = X[0:5]
        zd = X[6]

        # Run out the forward kinematics of the robot
        # to figure out where the foot is in world frame.
        foot_point = np.array([0.0, 0.0, -self.hopper_leg_length])
        foot_point_in_world = self.hopper.CalcPointsPositions(self.plant_context,
                                                              self.foot_frame, foot_point, self.world_frame)
        in_contact = foot_point_in_world[2] <= 0.01

        # Feel free to play with these values!
        # These should work pretty well for this problem set,
        # though.
        if (in_contact):
            if (zd > 0):
                # On the way back up,
                # "push" harder by increasing the effective
                # spring constant.
                l_rest = 1.05
            else:
                # On the way down,
                # "push" less hard by decreasing the effective
                # spring constant.
                l_rest = 1.0
        else:
            # Keep l_rest large to make sure the leg
            # is pushed back out to full extension quickly.
            l_rest = 1.0

        # See "Hopping in Legged Systems-Modeling and
        # Simulation for the Two-Dimensional One-Legged Case"
        # Section III for a great description of why
        # this works. (It has to do with a natural balance
        # arising between the energy lost from dissipation
        # during ground contact, and the energy injected by
        # this control.)

        return l_rest

    def DoCalcVectorOutput(self, context, u, x, y):
        # The naming if inputs is confusing, as this is a separate
        # system with its own state (x) and input (u), but the input
        # here is the state of the hopper.
        # Empty now
        if (self.print_period and
                context.get_time() - self.last_print_time >= self.print_period):
            print("t: ", context.get_time())
            self.last_print_time = context.get_time()

        # Update the internal context
        plant = self.hopper
        context = self.plant_context
        x_ref = plant.GetMutablePositionsAndVelocities(context)
        x_ref[:] = u

        # OK
        l_rest = 1.0  # self.ChooseSpringRestLength(X = u)

        # Passive spring force
        leg_compression_amount = l_rest - u[4]

        y[:] = [self.ChooseThighTorque(X=u),
                self.K_l * leg_compression_amount]


'''
Builds the block diagram for the 2d hopper
'''


def build_block_diagram(desired_lateral_velocity=0.0, print_period=0.0):
    builder = DiagramBuilder()

    # Build the plant
    plant = builder.AddSystem(MultibodyPlant(0.0005))
    scene_graph = builder.AddSystem(SceneGraph())
    plant.RegisterAsSourceForSceneGraph(scene_graph)
    builder.Connect(plant.get_geometry_poses_output_port(),
                    scene_graph.get_source_pose_port(
                        plant.get_source_id()))
    builder.Connect(scene_graph.get_query_output_port(),
                    plant.get_geometry_query_input_port())

    # Build the robot
    parser = Parser(plant)
    parser.AddModelFromFile("raibert_hopper_2d.sdf")
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("ground"))
    plant.Finalize()
    plant.set_name('plant')

    # Create a logger to log at 30hz
    state_dim = plant.num_positions() + plant.num_velocities()
    state_log = builder.AddSystem(SignalLogger(state_dim))
    state_log.DeclarePeriodicPublish(0.0333, 0.0)  # 30hz logging
    builder.Connect(plant.get_state_output_port(), state_log.get_input_port(0))
    state_log.set_name('state_log')

    # The controller
    controller = builder.AddSystem(
        Hopper2dController(plant,
                           desired_lateral_velocity=desired_lateral_velocity,
                           print_period=print_period))
    builder.Connect(plant.get_state_output_port(),
                    controller.get_input_port(0))
    builder.Connect(controller.get_output_port(
        0), plant.get_actuation_input_port())
    controller.set_name('controller')

    # Create visualizer
    visualizer = builder.AddSystem(PlanarSceneGraphVisualizer(
        scene_graph,
        xlim=[-1, 10],
        ylim=[-.2, 4.5],
        show=False
    ))
    builder.Connect(scene_graph.get_pose_bundle_output_port(),
                    visualizer.get_input_port(0))
    visualizer.set_name('visualizer')

    diagram = builder.Build()

    return diagram


'''
Simulates a 2d hopper from initial conditions x0 (which
should be a 10x1 np array) for duration seconds,
targeting a specified lateral velocity and printing to the
console every print_period seconds (as an indicator of
progress, only if print_period is nonzero).
'''


def Simulate2dHopper(x0, duration,
                     desired_lateral_velocity=0.0,
                     print_period=0.0):

    # The diagram, plant and contorller
    diagram = build_block_diagram(desired_lateral_velocity, print_period)

    # Start visualizer recording
    visualizer = diagram.GetSubsystemByName('visualizer')
    visualizer.start_recording()

    simulator = Simulator(diagram)
    simulator.Initialize()

    plant = diagram.GetSubsystemByName('plant')
    plant_context = diagram.GetMutableSubsystemContext(
        plant, simulator.get_mutable_context())
    plant_context.get_mutable_discrete_state_vector().SetFromVector(x0)

    potential = plant.CalcPotentialEnergy(plant_context)

    simulator.AdvanceTo(duration)

    visualizer.stop_recording()
    animation = visualizer.get_recording_as_animation()

    controller = diagram.GetSubsystemByName('controller')
    state_log = diagram.GetSubsystemByName('state_log')

    return plant, controller, state_log, animation


def Plot():
    # dummy controller just to build the diagram for the plot
    preload_controller = MatrixGain(np.zeros((1, 4)))
    preload_controller.set_name('preload controller')

    # plot block diagram
    plt.figure(figsize=(20, 8))
    diagram = build_block_diagram(preload_controller)
    diagram.set_name('Block Diagram of the One-Dimensional Hopper')
    plot_system_graphviz(diagram)


if __name__ == '__main__':
    x0 = np.zeros(10)
    x0[1] = 2
    x0[4] = 0.5
    hopper, controller, state_log = Simulate2dHopper(x0=x0,
                                                     duration=20,
                                                     desired_lateral_velocity=0.5,
                                                     print_period=1.0)
