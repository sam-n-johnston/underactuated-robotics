# -*- coding: utf8 -*-

import argparse
import math
import os.path
import time

import numpy as np

from pydrake.all import (
    RigidBodyTree,
    AddModelInstancesFromSdfString, FloatingBaseType,
    DiagramBuilder, 
    Simulator, VectorSystem,
    ConstantVectorSource, 
    SignalLogger,
    AbstractValue,
    Parser,
    PortDataType,
    MultibodyPlant,
    UniformGravityFieldElement
)
from IPython.display import HTML
import matplotlib.pyplot as plt
from drake import lcmt_viewer_load_robot
from pydrake.common.eigen_geometry import Quaternion
from pydrake.geometry import DispatchLoadMessage, SceneGraph
from pydrake.lcm import DrakeMockLcm
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.systems.rendering import PoseBundle

from underactuated.planar_multibody_visualizer import PlanarMultibodyVisualizer


class Hopper2dController(VectorSystem):
    def __init__(self, hopper, 
        desired_lateral_velocity = 0.0,
        print_period = 0.0):
        # hopper = a rigid body tree representing the 1D hopper
        # desired_lateral_velocity = How fast should the controller
        #  aim to run sideways?
        # print_period = print to console to indicate sim progress
        #  if nonzero
        VectorSystem.__init__(self,
            10, # 10 inputs: x, z, theta, alpha, l, and their derivatives
            2) # 2 outputs: Torque on thigh, and force on the leg extension
               #  link. (The passive spring for on the leg is calculated as
               #  part of this output.)
        self.hopper = hopper
        self.desired_lateral_velocity = desired_lateral_velocity
        self.print_period = print_period
        self.last_print_time = -print_period
        # Remember what the index of the foot is
        # self.foot_body_index = hopper.GetFrameByName("foot").get_body_index()
        self.foot_body_frame = hopper.GetFrameByName("foot")
        self.world_frame = hopper.world_frame()
        
        # The context for the hopper
        self.plant_context = self.hopper.CreateDefaultContext()

        # Default parameters for the hopper -- should match
        # raibert_hopper_1d.sdf, where applicable.
        # You're welcome to use these, but you probably don't need them.
        self.hopper_leg_length = 2.0
        self.body_size_height = 0.5
        mass_factor = 2.8052240
        self.m_b = 1.0 * mass_factor
        self.m_f = 0.1 * mass_factor
        self.l_max = 0.5
        self.gravity = 9.81

        # This is an arbitrary choice of spring constant for the leg.
        self.K_l = 100
        self.desired_alpha_array = []

    def calculate_energy_loss_by_stance_phase(self, touch_down_plus_state):
        return self.calculate_energy_loss_by_touch_down(touch_down_plus_state) + \
            self.calculate_energy_loss_by_lift_off(touch_down_plus_state)

    def calculate_kinetic_energy(self, mass, speed):
        return 1.0 / 2.0 * mass * speed ** 2.0

    def calculate_potential_energy(self, mass, height):
        return self.gravity * mass * height

    def spring_potential_energy(self, state):
        # TODO: use correct value
        l_rest = 1.0

        # Passive spring force
        leg_compression_amount = l_rest - state[4]
        return 1.0 / 2.0 * self.K_l * leg_compression_amount ** 2.0

    def potential_energy_body(self, state):
        return self.calculate_potential_energy(self.m_b, state[1])

    def potential_energy_foot(self, state):
        # Assuming foot's mass in a point mass in the middle
        # Not quite correct, need to account for alpha and theta
        is_foot_in_contact = state[1] + self.body_size_height - self.hopper_leg_length / 2.0 > self.hopper_leg_length / 2.0
        if is_foot_in_contact:
            foot_height = state[1] + self.body_size_height - self.hopper_leg_length / 2.0
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

    def calculate_energy_loss_by_touch_down(self, flight_phase):
        # Get total energy minus (before impact)
        total_energy_minus = self.calculate_total_energy(flight_phase)

        # Get touch down minus speeds (before impact)
        xd_minus = flight_phase[0+5]
        xd_minus_energy = self.calculate_kinetic_energy(self.m_b + self.m_f, xd_minus)
        # These are assuming that the leg is straight down.
        potential_energy_foot = self.calculate_potential_energy(self.m_b, self.hopper_leg_length / 2.0)
        potential_energy_body = self.calculate_potential_energy(self.m_b, self.hopper_leg_length - self.body_size_height)
        zd_energy = total_energy_minus - xd_minus_energy - \
            potential_energy_foot - potential_energy_body
        zd_minus = math.sqrt(2 * zd_energy / (self.m_b + self.m_f))

        kinetic_energy_lost_in_foot = self.calculate_kinetic_energy(self.m_f, xd_minus) + \
            self.calculate_kinetic_energy(self.m_f, zd_minus)

        return kinetic_energy_lost_in_foot

    def calculate_energy_loss_by_lift_off(self, flight_phase):
        # Get total energy minus (before impact)
        total_energy_minus = self.calculate_total_energy(flight_phase) - \
            self.calculate_energy_loss_by_touch_down(flight_phase)

        # Get lift off minus speeds (before impact)
        xd_minus = flight_phase[0+5] # Not quite right... but maybe close enough?
        xd_minus_energy = self.calculate_kinetic_energy(self.m_b, xd_minus)
        # Got to remove the potential energy from body + foot
        # These are assuming that the leg is straight down.
        potential_energy_foot = self.calculate_potential_energy(self.m_f, self.hopper_leg_length / 2.0)
        potential_energy_body = self.calculate_potential_energy(self.m_b, self.hopper_leg_length)

        zd_energy = total_energy_minus - xd_minus_energy - \
            potential_energy_foot - potential_energy_body
        zd_minus = math.sqrt(2 * zd_energy / self.m_b)

        # Calculate body speeds plus (after impact)
        xd_plus = self.m_b / (self.m_b + self.m_f) * xd_minus
        zd_plus = self.m_b / (self.m_b + self.m_f) * zd_minus

        # Calculate energy plus (after impact)
        spring_energy = 1.0 / 2.0 * self.K_l * (0.5) ** 2.0
        total_energy_plus = self.calculate_kinetic_energy(self.m_b + self.m_f, xd_plus) + \
            self.calculate_kinetic_energy(self.m_b + self.m_f, zd_plus) + \
                potential_energy_foot + potential_energy_body

        # Calculate energy loss
        return total_energy_minus - total_energy_plus

    def calculate_apex_xd_based_off_liftoff_plus(self, lift_off_plus_state):
        return lift_off_plus_state[0+5]

    def is_foot_in_contact(self, state):
        # Foot's mass might be a point mass in the MIDDLE!
        # Not quite correct, need to account for alpha and theta
        if state[1] + self.body_size_height - self.hopper_leg_length / 2.0 > 1.0:
            return False
        return True

    def calculate_apex_z_based_off_liftoff_plus(self, lift_off_plus_state):
        lift_off_zd = lift_off_plus_state[1+5]
        apex_z = lift_off_zd ** 2 / (2 * self.gravity)
        return apex_z + lift_off_plus_state[1]
         
    def calculate_moment_of_inertia(self):
        return self.m_f * self.l_max ** 2.

    def calculate_lift_off_angle(self):
        return math.atan2(self.desired_lateral_velocity, (2 * self.gravity * self.desired_height) ** (1. / 2.))
        
    def calculate_time_required_btwn_lo_and_td(self):
        lift_off_height = math.cos(self.calculate_lift_off_angle()) * self.hopper_leg_length 
        intial_vertical_speed = -(2 * self.gravity * self.desired_height) ** (1. / 2.)
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
                              self.foot_body_frame, foot_point, self.world_frame)
        in_contact = foot_point_in_world[2] <= 0.01
        in_air = foot_point_in_world[2] >= 0.1
        
        # It's all yours from here.
        # Implement a controller that:
        #  - Controls xd to self.desired_lateral_velocity
        #  - Attempts to keep the body steady (theta = 0)

        torque = self.PD_controller(X, in_contact, in_air)
                
        return torque
      
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
                              self.foot_body_frame, foot_point, self.world_frame)
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
            print "t: ", context.get_time()
            self.last_print_time = context.get_time()
        
        # Update the internal context
        plant = self.hopper
        context = self.plant_context
        x_ref = plant.GetMutablePositionsAndVelocities(context)
        x_ref[:] = u
        
        # OK
        l_rest = 1.0 # self.ChooseSpringRestLength(X = u)

        # Passive spring force
        leg_compression_amount = l_rest - u[4]

        y[:] = [  self.ChooseThighTorque(X = u),
                  self.K_l * leg_compression_amount]

'''
Simulates a 2d hopper from initial conditions x0 (which
should be a 10x1 np array) for duration seconds,
targeting a specified lateral velocity and printing to the
console every print_period seconds (as an indicator of
progress, only if print_period is nonzero).
'''
def Simulate2dHopper(x0, duration,
        desired_lateral_velocity = 0.0,
        print_period = 0.0):
    builder = DiagramBuilder()
    
    plant = builder.AddSystem(MultibodyPlant(0.0005))
    scene_graph = builder.AddSystem(SceneGraph())
    plant.RegisterAsSourceForSceneGraph(scene_graph)
    builder.Connect(plant.get_geometry_poses_output_port(),
                    scene_graph.get_source_pose_port(
                        plant.get_source_id()))
    builder.Connect(scene_graph.get_query_output_port(),
                    plant.get_geometry_query_input_port())
    
    # Build the plant
    parser = Parser(plant)
    parser.AddModelFromFile("raibert_hopper_2d.sdf")
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("ground"))
    plant.AddForceElement(UniformGravityFieldElement([0.0, 0.0, -9.81]))
    plant.Finalize()
    
    # Create a logger to log at 30hz
    state_dim = plant.num_positions() + plant.num_velocities()
    state_log = builder.AddSystem(SignalLogger(state_dim))
    state_log.DeclarePeriodicPublish(0.0333, 0.0) # 30hz logging
    builder.Connect(plant.get_state_output_port(), state_log.get_input_port(0))
    
    sams_controller = Hopper2dController(plant,
            desired_lateral_velocity = desired_lateral_velocity,
            print_period = print_period)
    desired_alpha = sams_controller.desired_alpha_array

    # The controller
    controller = builder.AddSystem(sams_controller)
    builder.Connect(plant.get_state_output_port(), controller.get_input_port(0))
    builder.Connect(controller.get_output_port(0), plant.get_actuation_input_port())

    # The diagram
    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.Initialize()
    
    plant_context = diagram.GetMutableSubsystemContext(
        plant, simulator.get_mutable_context())
    plant_context.get_mutable_discrete_state_vector().SetFromVector(x0)

    simulator.StepTo(duration)
    return plant, controller, state_log


def ConstructVisualizer():
    from underactuated import PlanarRigidBodyVisualizer
    tree = RigidBodyTree()
    AddModelInstancesFromSdfString(
        open("raibert_hopper_2d.sdf", 'r').read(),
        FloatingBaseType.kFixed,
        None, tree)
    viz = PlanarRigidBodyVisualizer(tree, xlim=[-5, 5], ylim=[-5, 5])
    viz.fig.set_size_inches(10, 5)
    return viz


if __name__ == '__main__':
    x0 = np.zeros(10)
    x0[1] = 2
    x0[4] = 0.5
    hopper, controller, state_log = Simulate2dHopper(x0 = x0,
                               duration=20,
                               desired_lateral_velocity = 0.5,
                               print_period = 1.0)