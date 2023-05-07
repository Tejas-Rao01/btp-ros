#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from math import sin, cos, pi, atan2
import numpy as np
from numpy import *


class ExtendedKalmanFilter:
    def __init__(self, state, covariance,
                 robot_width,
                 control_motion_factor, control_turn_factor):
        # The state. This is the core data of the Kalman filter.
        self.state = state
        self.covariance = covariance

        # Some constants.
        self.robot_width = robot_width
        self.control_motion_factor = control_motion_factor
        self.control_turn_factor = control_turn_factor

    @staticmethod
    def g(state, control, w):
        x, y, theta = state
        l, r = control
        if r != l:
            alpha = (r - l) / w
            rad = l/alpha
            g1 = x + (rad + w/2.)*(np.sin(theta+alpha) - np.sin(theta))
            g2 = y + (rad + w/2.)*(-np.cos(theta+alpha) + np.cos(theta))
            g3 = (theta + alpha + pi) % (2*pi) - pi
        else:
            g1 = x + l * np.cos(theta)
            g2 = y + l * np.sin(theta)
            g3 = theta

        return np.array([g1, g2, g3])

    @staticmethod
    def dg_dstate(state, control, w):

        theta = state[2]
        l, r = control

        if r != l:
            alpha = (r - l) / w
            R = l / alpha
            g1 = (R + (w / 2)) * (np.cos(theta + alpha) - np.cos(theta))
            g2 = (R + (w / 2)) * (np.sin(theta + alpha) - np.sin(theta))
            m = array([[1.0, 0.0, g1], [0.0, 1.0, g2], [0.0, 0.0, 1.0]])

        else:
            # This is for the special case r == l.
            m = np.array([[1.0, 0.0, -l * np.sin(theta)], [0.0, 1.0, l * np.cos(theta)], [0.0, 0.0, 1.0]])

        return m

    @staticmethod
    def dg_dcontrol(state, control, w):
        theta = state[2]
        l, r = tuple(control)

        if r != l:
            alpha = (r - l) / w

            wr = (w * r) / ((r - l) ** 2)
            wl = (w * l) / ((r - l) ** 2)
            r2l = (r + l) / (2 * (r - l))

            g1_l = wr * (sin(theta + alpha) - sin(theta)) - r2l * cos(theta + alpha)
            g2_l = wr * (-cos(theta + alpha) + cos(theta)) - r2l * sin(theta + alpha)
            g3_l = - (1 / w)

            g1_r = -wl * (sin(theta + alpha) - sin(theta)) + r2l * cos(theta + alpha)
            g2_r = -wl * (-cos(theta + alpha) + cos(theta)) + r2l * sin(theta + alpha)
            g3_r = 1 / w

            m = array([[g1_l, g1_r], [g2_l, g2_r], [g3_l, g3_r]])

        else:

            # This is for the special case l == r.
            g1_l = .5 * (np.cos(theta) + (l / w) * np.sin(theta))
            g2_l = .5 * (np.sin(theta) - (l / w) * np.cos(theta))
            g3_l = - 1 / w

            g1_r = .5 * ((-l / w) * np.sin(theta) + np.cos(theta))
            g2_r = .5 * ((l / w) * np.cos(theta) + np.sin(theta))
            g3_r = 1 / w

            m = np.array([[g1_l, g1_r], [g2_l, g2_r], [g3_l, g3_r]])

        return m

    @staticmethod
    def get_error_ellipse(covariance):
        """Return the position covariance (which is the upper 2x2 submatrix)
           as a triple: (main_axis_angle, stddev_1, stddev_2), where
           main_axis_angle is the angle (pointing direction) of the main axis,
           along which the standard deviation is stddev_1, and stddev_2 is the
           standard deviation along the other (orthogonal) axis."""
        eigenvals, eigenvects = linalg.eig(covariance[0:2,0:2])
        angle = atan2(eigenvects[1,0], eigenvects[0,0])
        return (angle, sqrt(eigenvals[0]), sqrt(eigenvals[1]))        

    def predict(self, control):
        """The prediction step of the Kalman filter."""
        # covariance' = G * covariance * GT + R
        # where R = V * (covariance in control space) * VT.
        # Covariance in control space depends on move distance.
        left, right = control

        # --->>> Put your code to compute the new self.covariance here.
        # First, construct the control_covariance, which is a diagonal matrix.
        # In Python/Numpy, you may use diag([a, b]) to get
        # [[ a, 0 ],
        #  [ 0, b ]].
        # Then, compute G using dg_dstate and V using dg_dcontrol.
        # Then, compute the new self.covariance.
        # Note that the transpose of a Numpy array G is expressed as G.T,
        # and the matrix product of A and B is written as dot(A, B).
        # Writing A*B instead will give you the element-wise product, which
        # is not intended here.

        alpha_1 = self.control_motion_factor
        alpha_2 = self.control_turn_factor

        sigma_l = (alpha_1*left) + (alpha_2*(left - right))**2
        sigma_r = (alpha_1*right)+ (alpha_2*(left - right))**2
        control_covariance = np.diag([sigma_l, sigma_r])

        G_t = self.dg_dstate(self.state, control, self.robot_width)
        V = self.dg_dcontrol(self.state, control, self.robot_width)

        self.covariance = np.dot(G_t, np.dot(self.covariance, G_t.T)) + np.dot(V, np.dot(control_covariance, V.T))

        # --->>> Put your code to compute the new self.state here.
        self.state = self.g(self.state, control, self.robot_width)

if __name__ == '__main__':
    # Robot constants.
    scanner_displacement = 0

    robot_width = 0.4

    # Filter constants.
    control_motion_factor = 0.5  # Error in motor control.
    control_turn_factor = 0.0  # Additional error due to slip when turning.

    # Measured start position.
    initial_state = array([0,0,0])
    # Covariance at start position.
    initial_covariance = diag([1, 1, 1])
    # Setup filter.
    kf = ExtendedKalmanFilter(initial_state, initial_covariance,
                              robot_width,
                              control_motion_factor, control_turn_factor)

    kf.predict([1,1])
    print(kf.covariance)

