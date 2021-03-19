import numpy as np
import math
from math import sin, cos, tan, sqrt
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
from mpl_toolkits.mplot3d import Axes3D
import random

from solution import UDACITYDroneIn3D, UDACITYController
import testing


class DroneIn3D(UDACITYDroneIn3D):

    def __init__(self,
                 k_f=1.0,
                 k_m=1.0,
                 m=0.5,
                 L=0.566,  # full rotor to rotor distance
                 i_x=0.1,
                 i_y=0.1,
                 i_z=0.2):
        self.k_f = k_f
        self.k_m = k_m
        self.m = m
        self.l = L / (2 * sqrt(2))  # perpendicular distance to axes
        self.i_x = i_x
        self.i_y = i_y
        self.i_z = i_z

        # x, y, y, phi, theta, psi,
        # x_dot, y_dot, z_dot, p, q, r
        self.X = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.omega = np.array([0.0, 0.0, 0.0, 0.0])

        self.g = 9.81

    @property
    def phi(self):
        return self.X[3]

    @property
    def theta(self):
        return self.X[4]

    @property
    def psi(self):
        return self.X[5]

    # body rates [rad/s] (in body frame)
    @property
    def p(self):
        return self.X[9]

    @property
    def q(self):
        return self.X[10]

    @property
    def r(self):
        return self.X[11]

    # Forces from the four propellers
    @property
    def f_1(self):
        f = self.k_f * self.omega[0] ** 2
        return f

    @property
    def f_2(self):
        f = self.k_f * self.omega[1] ** 2
        return f

    @property
    def f_3(self):
        f = self.k_f * self.omega[2] ** 2
        return f

    @property
    def f_4(self):
        f = self.k_f * self.omega[3] ** 2
        return f

    # collective force
    @property
    def f_total(self):
        f_t = self.f_1 + self.f_2 + self.f_3 + self.f_4
        return f_t

    # propellers 1 and 3 rotate in clockwise thus producing the moment in the counterclockwise direction with negative sign
    # propellers 2 and 4 rotate in counterclockwise thus the resulting moments are in opposite and have the positive sign

    # check 1.1
    @property
    def tau_1(self):
        tau = -self.k_m * self.omega[0] ** 2
        return tau

    @property
    def tau_2(self):
        tau = self.k_m * self.omega[1] ** 2
        return tau

    @property
    def tau_3(self):
        tau = -self.k_m * self.omega[2] ** 2
        return tau

    @property
    def tau_4(self):
        tau = self.k_m * self.omega[3] ** 2
        return tau

    @property
    def tau_x(self):
        tau = self.l * (self.f_1 + self.f_4 - self.f_2 - self.f_3)
        return tau

    @property
    def tau_y(self):
        tau = self.l * (self.f_1 + self.f_2 - self.f_3 - self.f_4)
        return tau

    @property
    def tau_z(self):
        tau = self.tau_1 + self.tau_2 + self.tau_3 + self.tau_4
        return tau

    def set_propeller_angular_velocities(self,
                                         c,
                                         u_bar_p,
                                         u_bar_q,
                                         u_bar_r):
        # TODO replace with your own implementation.
        #   note that this function doesn't return anything
        #   it just sets self.omega
        #
        # self.omega[0] =
        # self.omega[1] =
        # self.omega[2] =
        # self.omega[3] =

        c_bar = -c * self.m / self.k_f
        p_bar = (self.i_x * u_bar_p) / (self.k_f * self.l)
        q_bar = (self.i_y * u_bar_q) / (self.k_f * self.l)
        r_bar = (self.i_z * u_bar_r) / self.k_m

        omega_4 = (c_bar + p_bar - r_bar - q_bar) / 4
        omega_3 = (r_bar - p_bar) / 2 + omega_4
        omega_2 = (c_bar - p_bar) / 2 - omega_3
        omega_1 = c_bar - omega_2 - omega_3 - omega_4

        self.omega[0] = -np.sqrt(omega_1)
        self.omega[1] = np.sqrt(omega_2)
        self.omega[2] = -np.sqrt(omega_3)
        self.omega[3] = np.sqrt(omega_4)


testing.test_exercise_1_1(DroneIn3D)
