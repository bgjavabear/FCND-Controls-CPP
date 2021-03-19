import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab
import jdc
from ExerciseAnswers import Answers


class Drone2D:

    def __init__(self,
                 k_f=0.1,  # value of the thrust coefficient
                 i=0.1,  # moment of inertia around the x-axis
                 m=1.0,  # mass of the vehicle
                 l=0.15,  # distance between the center of
                 #   mass and the propeller axis
                 ):
        self.k_f = k_f
        self.i = i
        self.l = l
        self.m = m

        self.omega_1 = 0.0
        self.omega_2 = 0.0
        self.g = 9.81

        # z, y, phi, z_dot, y_dot, phi_dot
        self.X = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def advance_state_uncontrolled(self, dt):
        """Advances the state of the drone by dt seconds.
        Note that this method assumes zero rotational speed
        for both propellers."""

        X_dot = np.array([
            self.X[3],
            self.X[4],
            self.X[5],
            self.g,
            0,
            0
        ])

        self.X = self.X + X_dot * dt

        return self.X
