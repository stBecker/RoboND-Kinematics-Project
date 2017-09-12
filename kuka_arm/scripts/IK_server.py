#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

import numpy as np


def Rot(axis, q):
    if axis == 'Z':
        R = Matrix([[cos(q), -sin(q), 0, 0],
                    [sin(q), cos(q), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1],
                    ])

    elif axis == 'Y':
        R = Matrix([[cos(q), 0, sin(q), 0],
                    [0, 1, 0, 0],
                    [-sin(q), 0, cos(q), 0],
                    [0, 0, 0, 1],
                    ])

    elif axis == 'X':
        R = Matrix([[1, 0, 0, 0],
                    [0, cos(q), -sin(q), 0],
                    [0, sin(q), cos(q), 0],
                    [0, 0, 0, 1],
                    ])

    # if axis == 'Z':
    #     R = Matrix([[cos(q), -sin(q), 0],
    #                 [sin(q), cos(q), 0],
    #                 [0, 0, 1]])
    #
    # elif axis == 'Y':
    #     R = Matrix([[cos(q), 0, sin(q)],
    #                 [0, 1, 0],
    #                 [-sin(q), 0, cos(q)]])
    #
    # elif axis == 'X':
    #     R = Matrix([[1, 0, 0],
    #                 [0, cos(q), -sin(q)],
    #                 [0, sin(q), cos(q)]])

    else:
        raise ValueError

    return R


def get_pre_calculated_transforms():
    """
    Matrix([[1.00000000000000, 0, 0, 0], [0, 1.00000000000000, 0, 0], [0, 0, 1.00000000000000, 0.750000000000000], [0, 0, 0, 1.00000000000000]])
    Matrix([[0, 1.00000000000000, 0, 0.350000000000000], [0, 0, 1.00000000000000, 0], [1.00000000000000, 0, 0, 0.750000000000000], [0, 0, 0, 1.00000000000000]])
    Matrix([[0, 1.00000000000000, 0, 0.350000000000000], [0, 0, 1.00000000000000, 0], [1.00000000000000, 0, 0, 2.00000000000000], [0, 0, 0, 1.00000000000000]])
    Matrix([[0, 0, 1.00000000000000, 1.85000000000000], [0, -1.00000000000000, 0, 0], [1.00000000000000, 0, 0, 1.94600000000000], [0, 0, 0, 1.00000000000000]])
    Matrix([[0, 1.00000000000000, 0, 1.85000000000000], [0, 0, 1.00000000000000, 0], [1.00000000000000, 0, 4.00000000000000, 1.94600000000000], [0, 0, 0, 1.00000000000000]])
    Matrix([[0, 0, 1.00000000000000, 1.85000000000000], [0, -1.00000000000000, 0, 0], [1.00000000000000, -4.00000000000000, 5.00000000000000, 1.94600000000000], [0, 0, 0, 1.00000000000000]])
    Matrix([[0, 0, 1.00000000000000, 2.15300000000000], [0, -1.00000000000000, 0, 0], [1.00000000000000, -4.00000000000000, 11.0000000000000, 3.46100000000000], [0, 0, 0, 1.00000000000000]])
    Matrix([[1.00000000000000, 0, 0, 2.15300000000000], [0, 1.00000000000000, 0, 0], [11.0000000000000, 4.00000000000000, 1.00000000000000, 3.46100000000000], [0, 0, 0, 1.00000000000000]])

    """
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')  # theta_i
    T0_2 = Matrix([[sin(q2) * cos(q1), cos(q1) * cos(q2), -sin(q1), 0.35 * cos(q1)],
                   [sin(q1) * sin(q2), sin(q1) * cos(q2), cos(q1), 0.35 * sin(q1)],
                   [cos(q2), -sin(q2), 0, 0.750000000000000],
                   [0, 0, 0, 1]])
    T0_3 = Matrix([[sin(q2 + q3) * cos(q1), cos(q1) * cos(q2 + q3), -sin(q1), (1.25 * sin(q2) + 0.35) * cos(q1)],
                   [sin(q1) * sin(q2 + q3), sin(q1) * cos(q2 + q3), cos(q1), (1.25 * sin(q2) + 0.35) * sin(q1)],
                   [cos(q2 + q3), -sin(q2 + q3), 0, 1.25 * cos(q2) + 0.75], [0, 0, 0, 1]])
    T0_4 = Matrix(
        [[sin(q1) * sin(q4) + sin(q2 + q3) * cos(q1) * cos(q4), sin(q1) * cos(q4) - sin(q4) * sin(q2 + q3) * cos(q1),
          cos(q1) * cos(q2 + q3), (1.25 * sin(q2) - 0.054 * sin(q2 + q3) + 1.5 * cos(q2 + q3) + 0.35) * cos(q1)],
         [sin(q1) * sin(q2 + q3) * cos(q4) - sin(q4) * cos(q1),
          -sin(q1) * sin(q4) * sin(q2 + q3) - cos(q1) * cos(q4), sin(q1) * cos(q2 + q3),
          (1.25 * sin(q2) - 0.054 * sin(q2 + q3) + 1.5 * cos(q2 + q3) + 0.35) * sin(q1)],
         [cos(q4) * cos(q2 + q3), -sin(q4) * cos(q2 + q3), -sin(q2 + q3),
          -1.5 * sin(q2 + q3) + 1.25 * cos(q2) - 0.054 * cos(q2 + q3) + 0.75], [0, 0, 0, 1]])
    T0_5 = Matrix([[(sin(q1) * sin(q4) + sin(q2 + q3) * cos(q1) * cos(q4)) * cos(q5) + sin(q5) * cos(q1) * cos(q2 + q3),
                    -(sin(q1) * sin(q4) + sin(q2 + q3) * cos(q1) * cos(q4)) * sin(q5) + cos(q1) * cos(q5) * cos(
                        q2 + q3),
                    4 * sin(q1) * sin(q4) - sin(q1) * cos(q4) + sin(q4) * sin(q2 + q3) * cos(q1) + 4 * sin(
                        q2 + q3) * cos(
                        q1) * cos(q4), (1.25 * sin(q2) - 0.054 * sin(q2 + q3) + 1.5 * cos(q2 + q3) + 0.35) * cos(q1)],
                   [(sin(q1) * sin(q2 + q3) * cos(q4) - sin(q4) * cos(q1)) * cos(q5) + sin(q1) * sin(q5) * cos(q2 + q3),
                    -(sin(q1) * sin(q2 + q3) * cos(q4) - sin(q4) * cos(q1)) * sin(q5) + sin(q1) * cos(q5) * cos(
                        q2 + q3),
                    sin(q1) * sin(q4) * sin(q2 + q3) + 4 * sin(q1) * sin(q2 + q3) * cos(q4) - 4 * sin(q4) * cos(
                        q1) + cos(
                        q1) * cos(q4), (1.25 * sin(q2) - 0.054 * sin(q2 + q3) + 1.5 * cos(q2 + q3) + 0.35) * sin(q1)],
                   [-sin(q5) * sin(q2 + q3) + cos(q4) * cos(q5) * cos(q2 + q3),
                    -sin(q5) * cos(q4) * cos(q2 + q3) - sin(q2 + q3) * cos(q5), (sin(q4) + 4 * cos(q4)) * cos(q2 + q3),
                    -1.5 * sin(q2 + q3) + 1.25 * cos(q2) - 0.054 * cos(q2 + q3) + 0.75], [0, 0, 0, 1]])
    T0_6 = Matrix([[(
                        (sin(q1) * sin(q4) + sin(q2 + q3) * cos(q1) * cos(q4)) * cos(q5) + sin(q5) * cos(q1) * cos(
                            q2 + q3)) * cos(
        q6) - (4 * sin(q1) * sin(q4) - sin(q1) * cos(q4) + sin(q4) * sin(q2 + q3) * cos(q1) + 4 * sin(q2 + q3) * cos(
        q1) * cos(q4)) * sin(q6), -(
        (sin(q1) * sin(q4) + sin(q2 + q3) * cos(q1) * cos(q4)) * cos(q5) + sin(q5) * cos(q1) * cos(q2 + q3)) * sin(
        q6) - (
                        4 * sin(q1) * sin(q4) - sin(q1) * cos(q4) + sin(q4) * sin(q2 + q3) * cos(q1) + 4 * sin(
                            q2 + q3) * cos(
                            q1) * cos(q4)) * cos(q6),
                    -(sin(q1) * sin(q4) + sin(q2 + q3) * cos(q1) * cos(q4)) * sin(q5) + 5 * (
                        sin(q1) * sin(q4) + sin(q2 + q3) * cos(q1) * cos(q4)) * cos(q5) + 5 * sin(q5) * cos(q1) * cos(
                        q2 + q3) + cos(q1) * cos(q5) * cos(q2 + q3),
                    (1.25 * sin(q2) - 0.054 * sin(q2 + q3) + 1.5 * cos(q2 + q3) + 0.35) * cos(q1)], [((sin(q1) * sin(
        q2 + q3) * cos(q4) - sin(q4) * cos(q1)) * cos(q5) + sin(q1) * sin(q5) * cos(q2 + q3)) * cos(q6) - (
                                                                                                         sin(q1) * sin(
                                                                                                             q4) * sin(
                                                                                                             q2 + q3) + 4 * sin(
                                                                                                             q1) * sin(
                                                                                                             q2 + q3) * cos(
                                                                                                             q4) - 4 * sin(
                                                                                                             q4) * cos(
                                                                                                             q1) + cos(
                                                                                                             q1) * cos(
                                                                                                             q4)) * sin(
        q6), -(
        (sin(q1) * sin(q2 + q3) * cos(q4) - sin(q4) * cos(q1)) * cos(q5) + sin(q1) * sin(q5) * cos(q2 + q3)) * sin(
        q6) - (
                                                                                                         sin(q1) * sin(
                                                                                                             q4) * sin(
                                                                                                             q2 + q3) + 4 * sin(
                                                                                                             q1) * sin(
                                                                                                             q2 + q3) * cos(
                                                                                                             q4) - 4 * sin(
                                                                                                             q4) * cos(
                                                                                                             q1) + cos(
                                                                                                             q1) * cos(
                                                                                                             q4)) * cos(
        q6), -(sin(q1) * sin(q2 + q3) * cos(q4) - sin(q4) * cos(q1)) * sin(q5) + 5 * (sin(q1) * sin(q2 + q3) * cos(
        q4) - sin(q4) * cos(q1)) * cos(q5) + 5 * sin(q1) * sin(q5) * cos(q2 + q3) + sin(q1) * cos(q5) * cos(q2 + q3), (
                                                                                                         1.25 * sin(
                                                                                                             q2) - 0.054 * sin(
                                                                                                             q2 + q3) + 1.5 * cos(
                                                                                                             q2 + q3) + 0.35) * sin(
        q1)], [-(sin(q5) * sin(q2 + q3) - cos(q4) * cos(q5) * cos(q2 + q3)) * cos(q6) - (sin(q4) + 4 * cos(q4)) * sin(
        q6) * cos(q2 + q3),
               (sin(q5) * sin(q2 + q3) - cos(q4) * cos(q5) * cos(q2 + q3)) * sin(q6) - (sin(q4) + 4 * cos(q4)) * cos(
                   q6) * cos(q2 + q3),
               -5 * sin(q5) * sin(q2 + q3) - sin(q5) * cos(q4) * cos(q2 + q3) - sin(q2 + q3) * cos(q5) + 5 * cos(
                   q4) * cos(q5) * cos(q2 + q3), -1.5 * sin(q2 + q3) + 1.25 * cos(q2) - 0.054 * cos(q2 + q3) + 0.75],
                   [0, 0, 0, 1]])
    T0_G = Matrix([[(
                        (sin(q1) * sin(q4) + sin(q2 + q3) * cos(q1) * cos(q4)) * cos(q5) + sin(q5) * cos(q1) * cos(
                            q2 + q3)) * cos(
        q6) - (4 * sin(q1) * sin(q4) - sin(q1) * cos(q4) + sin(q4) * sin(q2 + q3) * cos(q1) + 4 * sin(q2 + q3) * cos(
        q1) * cos(q4)) * sin(q6), -(
        (sin(q1) * sin(q4) + sin(q2 + q3) * cos(q1) * cos(q4)) * cos(q5) + sin(q5) * cos(q1) * cos(q2 + q3)) * sin(
        q6) - (
                        4 * sin(q1) * sin(q4) - sin(q1) * cos(q4) + sin(q4) * sin(q2 + q3) * cos(q1) + 4 * sin(
                            q2 + q3) * cos(
                            q1) * cos(q4)) * cos(q6), 6 * (
                        (sin(q1) * sin(q4) + sin(q2 + q3) * cos(q1) * cos(q4)) * cos(q5) + sin(q5) * cos(q1) * cos(
                            q2 + q3)) * cos(
        q6) - (sin(q1) * sin(q4) + sin(q2 + q3) * cos(q1) * cos(q4)) * sin(q5) + 5 * (
                        sin(q1) * sin(q4) + sin(q2 + q3) * cos(q1) * cos(q4)) * cos(q5) - 6 * (
                        4 * sin(q1) * sin(q4) - sin(q1) * cos(q4) + sin(q4) * sin(q2 + q3) * cos(q1) + 4 * sin(
                            q2 + q3) * cos(
                            q1) * cos(q4)) * sin(q6) + 5 * sin(q5) * cos(q1) * cos(q2 + q3) + cos(q1) * cos(q5) * cos(
        q2 + q3),
                    -0.303 * (sin(q1) * sin(q4) + sin(q2 + q3) * cos(q1) * cos(q4)) * sin(q5) + 1.515 * (
                        sin(q1) * sin(q4) + sin(q2 + q3) * cos(q1) * cos(q4)) * cos(q5) + (
                        1.25 * sin(q2) - 0.054 * sin(q2 + q3) + 1.5 * cos(q2 + q3) + 0.35) * cos(q1) + 1.515 * sin(
                        q5) * cos(
                        q1) * cos(q2 + q3) + 0.303 * cos(q1) * cos(q5) * cos(q2 + q3)], [((sin(q1) * sin(q2 + q3) * cos(
        q4) - sin(q4) * cos(q1)) * cos(q5) + sin(q1) * sin(q5) * cos(q2 + q3)) * cos(q6) - (sin(q1) * sin(q4) * sin(
        q2 + q3) + 4 * sin(q1) * sin(q2 + q3) * cos(q4) - 4 * sin(q4) * cos(q1) + cos(q1) * cos(q4)) * sin(q6), -(
        (sin(q1) * sin(q2 + q3) * cos(q4) - sin(q4) * cos(q1)) * cos(q5) + sin(q1) * sin(q5) * cos(q2 + q3)) * sin(
        q6) - (
                                                                                             sin(q1) * sin(q4) * sin(
                                                                                                 q2 + q3) + 4 * sin(
                                                                                                 q1) * sin(
                                                                                                 q2 + q3) * cos(
                                                                                                 q4) - 4 * sin(
                                                                                                 q4) * cos(q1) + cos(
                                                                                                 q1) * cos(
                                                                                                 q4)) * cos(q6),
                                                                                         6 * ((sin(
                                                                                             q1) * sin(q2 + q3) * cos(
                                                                                             q4) - sin(q4) * cos(
                                                                                             q1)) * cos(q5) + sin(
                                                                                             q1) * sin(q5) * cos(
                                                                                             q2 + q3)) * cos(q6) - (
                                                                                             sin(q1) * sin(
                                                                                                 q2 + q3) * cos(
                                                                                                 q4) - sin(q4) * cos(
                                                                                                 q1)) * sin(
                                                                                             q5) + 5 * (
                                                                                         sin(q1) * sin(q2 + q3) * cos(
                                                                                             q4) - sin(q4) * cos(
                                                                                             q1)) * cos(q5) - 6 * (
                                                                                         sin(q1) * sin(q4) * sin(
                                                                                             q2 + q3) + 4 * sin(
                                                                                             q1) * sin(q2 + q3) * cos(
                                                                                             q4) - 4 * sin(q4) * cos(
                                                                                             q1) + cos(q1) * cos(
                                                                                             q4)) * sin(
                                                                                             q6) + 5 * sin(q1) * sin(
                                                                                             q5) * cos(q2 + q3) + sin(
                                                                                             q1) * cos(q5) * cos(
                                                                                             q2 + q3), -0.303 * (
                                                                                             sin(q1) * sin(
                                                                                                 q2 + q3) * cos(
                                                                                                 q4) - sin(q4) * cos(
                                                                                                 q1)) * sin(
            q5) + 1.515 * (sin(q1) * sin(q2 + q3) * cos(q4) - sin(q4) * cos(q1)) * cos(q5) + (
                                                                                         1.25 * sin(q2) - 0.054 * sin(
                                                                                             q2 + q3) + 1.5 * cos(
                                                                                             q2 + q3) + 0.35) * sin(
            q1) + 1.515 * sin(q1) * sin(q5) * cos(q2 + q3) + 0.303 * sin(
            q1) * cos(q5) * cos(q2 + q3)], [
                       -(sin(q5) * sin(q2 + q3) - cos(q4) * cos(q5) * cos(q2 + q3)) * cos(q6) - (
                       sin(q4) + 4 * cos(q4)) * sin(
                           q6) * cos(q2 + q3),
                       (sin(q5) * sin(q2 + q3) - cos(q4) * cos(q5) * cos(q2 + q3)) * sin(q6) - (
                       sin(q4) + 4 * cos(q4)) * cos(
                           q6) * cos(q2 + q3),
                       -6 * (sin(q5) * sin(q2 + q3) - cos(q4) * cos(q5) * cos(q2 + q3)) * cos(q6) - 6 * (
                           sin(q4) + 4 * cos(q4)) * sin(q6) * cos(q2 + q3) - 5 * sin(q5) * sin(q2 + q3) - sin(q5) * cos(
                           q4) * cos(
                           q2 + q3) - sin(q2 + q3) * cos(q5) + 5 * cos(q4) * cos(q5) * cos(q2 + q3),
                       -1.515 * sin(q5) * sin(q2 + q3) - 0.303 * sin(q5) * cos(q4) * cos(q2 + q3) - 0.303 * sin(
                           q2 + q3) * cos(
                           q5) - 1.5 * sin(q2 + q3) + 1.25 * cos(q2) + 1.515 * cos(q4) * cos(q5) * cos(
                           q2 + q3) - 0.054 * cos(
                           q2 + q3) + 0.75], [0, 0, 0, 1]])
    T_total = Matrix([[6 * (
        (sin(q1) * sin(q4) + sin(q2 + q3) * cos(q1) * cos(q4)) * cos(q5) + sin(q5) * cos(q1) * cos(q2 + q3)) * cos(
        q6) - (
                           sin(q1) * sin(q4) + sin(q2 + q3) * cos(q1) * cos(q4)) * sin(q5) + 5 * (
                           sin(q1) * sin(q4) + sin(q2 + q3) * cos(q1) * cos(q4)) * cos(q5) - 6 * (
                           4 * sin(q1) * sin(q4) - sin(q1) * cos(q4) + sin(q4) * sin(q2 + q3) * cos(q1) + 4 * sin(
                               q2 + q3) * cos(
                               q1) * cos(q4)) * sin(q6) + 5 * sin(q5) * cos(q1) * cos(q2 + q3) + cos(q1) * cos(
        q5) * cos(q2 + q3), (
                           (sin(q1) * sin(q4) + sin(q2 + q3) * cos(q1) * cos(q4)) * cos(q5) + sin(q5) * cos(q1) * cos(
                               q2 + q3)) * sin(
        q6) + (4 * sin(q1) * sin(q4) - sin(q1) * cos(q4) + sin(q4) * sin(q2 + q3) * cos(q1) + 4 * sin(q2 + q3) * cos(
        q1) * cos(q4)) * cos(q6), (
                           (sin(q1) * sin(q4) + sin(q2 + q3) * cos(q1) * cos(q4)) * cos(q5) + sin(q5) * cos(q1) * cos(
                               q2 + q3)) * cos(
        q6) - (4 * sin(q1) * sin(q4) - sin(q1) * cos(q4) + sin(q4) * sin(q2 + q3) * cos(q1) + 4 * sin(q2 + q3) * cos(
        q1) * cos(q4)) * sin(q6), -0.303 * (sin(q1) * sin(q4) + sin(q2 + q3) * cos(q1) * cos(q4)) * sin(q5) + 1.515 * (
                           sin(q1) * sin(q4) + sin(q2 + q3) * cos(q1) * cos(q4)) * cos(q5) + (
                           1.25 * sin(q2) - 0.054 * sin(q2 + q3) + 1.5 * cos(q2 + q3) + 0.35) * cos(q1) + 1.515 * sin(
        q5) * cos(
        q1) * cos(q2 + q3) + 0.303 * cos(q1) * cos(q5) * cos(q2 + q3)], [6 * (
        (sin(q1) * sin(q2 + q3) * cos(q4) - sin(q4) * cos(q1)) * cos(q5) + sin(q1) * sin(q5) * cos(q2 + q3)) * cos(
        q6) - (
                                                                             sin(q1) * sin(q2 + q3) * cos(q4) - sin(
                                                                                 q4) * cos(q1)) * sin(q5) + 5 * (
                                                                             sin(q1) * sin(q2 + q3) * cos(q4) - sin(
                                                                                 q4) * cos(q1)) * cos(q5) - 6 * (
                                                                             sin(q1) * sin(q4) * sin(q2 + q3) + 4 * sin(
                                                                                 q1) * sin(q2 + q3) * cos(q4) - 4 * sin(
                                                                                 q4) * cos(q1) + cos(q1) * cos(
                                                                                 q4)) * sin(
        q6) + 5 * sin(q1) * sin(q5) * cos(q2 + q3) + sin(q1) * cos(q5) * cos(q2 + q3), ((sin(q1) * sin(q2 + q3) * cos(
        q4) - sin(q4) * cos(q1)) * cos(q5) + sin(q1) * sin(q5) * cos(q2 + q3)) * sin(q6) + (
                                                                             sin(q1) * sin(q4) * sin(q2 + q3) + 4 * sin(
                                                                                 q1) * sin(q2 + q3) * cos(q4) - 4 * sin(
                                                                                 q4) * cos(q1) + cos(q1) * cos(
                                                                                 q4)) * cos(
        q6), ((sin(q1) * sin(q2 + q3) * cos(q4) - sin(q4) * cos(q1)) * cos(q5) + sin(q1) * sin(q5) * cos(
        q2 + q3)) * cos(q6) - (sin(q1) * sin(q4) * sin(q2 + q3) + 4 * sin(q1) * sin(q2 + q3) * cos(q4) - 4 * sin(
        q4) * cos(q1) + cos(q1) * cos(q4)) * sin(q6), -0.303 * (sin(q1) * sin(q2 + q3) * cos(q4) - sin(q4) * cos(
        q1)) * sin(q5) + 1.515 * (sin(q1) * sin(q2 + q3) * cos(q4) - sin(q4) * cos(q1)) * cos(q5) + (
                                                                             1.25 * sin(q2) - 0.054 * sin(
                                                                                 q2 + q3) + 1.5 * cos(
                                                                                 q2 + q3) + 0.35) * sin(
        q1) + 1.515 * sin(
        q1) * sin(q5) * cos(q2 + q3) + 0.303 * sin(q1) * cos(q5) * cos(q2 + q3)], [
                          -6 * (sin(q5) * sin(q2 + q3) - cos(q4) * cos(q5) * cos(q2 + q3)) * cos(q6) - 6 * (
                              sin(q4) + 4 * cos(q4)) * sin(q6) * cos(q2 + q3) - 5 * sin(q5) * sin(q2 + q3) - sin(
                              q5) * cos(q4) * cos(
                              q2 + q3) - sin(q2 + q3) * cos(q5) + 5 * cos(q4) * cos(q5) * cos(q2 + q3),
                          -(sin(q5) * sin(q2 + q3) - cos(q4) * cos(q5) * cos(q2 + q3)) * sin(q6) + (
                          sin(q4) + 4 * cos(q4)) * cos(
                              q6) * cos(q2 + q3),
                          -(sin(q5) * sin(q2 + q3) - cos(q4) * cos(q5) * cos(q2 + q3)) * cos(q6) - (
                          sin(q4) + 4 * cos(q4)) * sin(
                              q6) * cos(q2 + q3),
                          -1.515 * sin(q5) * sin(q2 + q3) - 0.303 * sin(q5) * cos(q4) * cos(q2 + q3) - 0.303 * sin(
                              q2 + q3) * cos(
                              q5) - 1.5 * sin(q2 + q3) + 1.25 * cos(q2) + 1.515 * cos(q4) * cos(q5) * cos(
                              q2 + q3) - 0.054 * cos(
                              q2 + q3) + 0.75], [0, 0, 0, 1]])

    R_corr = Matrix([[0, 0, 1, 0], [0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1]])

    R0_3 = T0_3[0:3, 0:3]
    R0_6 = T0_6[0:3, 0:3]

    return R_corr, R0_3, R0_6


def calculate_forward_kinematics():
    ### Your FK code here
    # Create symbols
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')  # theta_i
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

    # Create Modified DH parameters
    s = {
        alpha0: 0, a0: 0, d1: 0.75,
        alpha1: -pi / 2, a1: 0.35, d2: 0, q2: q2 - pi / 2,
        alpha2: 0, a2: 1.25, d3: 0,
        alpha3: -pi / 2, a3: -0.054, d4: 1.50,
        alpha4: pi / 2, a4: 0, d5: 0,
        alpha5: -pi / 2, a5: 0, d6: 0,
        alpha6: 0, a6: 0, d7: 0.303, q7: 0,
    }

    # Define Modified DH Transformation matrix
    #### Homogeneous Transforms
    T0_1 = Matrix([[cos(q1), -sin(q1), 0, a0],
                   [sin(q1) * cos(alpha0), cos(q1) * cos(alpha0), -sin(alpha0), -sin(alpha0) * d1],
                   [sin(q1) * sin(alpha0), cos(q1) * sin(alpha0), cos(alpha0), cos(alpha0) * d1],
                   [0, 0, 0, 1]])
    T0_1 = T0_1.subs(s)

    T1_2 = Matrix([[cos(q2), -sin(q2), 0, a1],
                   [sin(q2) * cos(alpha1), cos(q2) * cos(alpha1), -sin(alpha1), -sin(alpha1) * d2],
                   [sin(q2) * sin(alpha1), cos(q2) * sin(alpha1), cos(alpha1), cos(alpha1) * d2],
                   [0, 0, 0, 1]])
    T1_2 = T1_2.subs(s)

    T2_3 = Matrix([[cos(q3), -sin(q3), 0, a2],
                   [sin(q3) * cos(alpha2), cos(q3) * cos(alpha2), -sin(alpha2), -sin(alpha2) * d3],
                   [sin(q3) * sin(alpha2), cos(q3) * sin(alpha2), cos(alpha2), cos(alpha2) * d3],
                   [0, 0, 0, 1]])
    T2_3 = T2_3.subs(s)

    T3_4 = Matrix([[cos(q4), -sin(q4), 0, a3],
                   [sin(q4) * cos(alpha3), cos(q4) * cos(alpha3), -sin(alpha3), -sin(alpha3) * d4],
                   [sin(q4) * sin(alpha3), cos(q4) * sin(alpha3), cos(alpha3), cos(alpha3) * d4],
                   [0, 0, 0, 1]])
    T3_4 = T3_4.subs(s)

    T4_5 = Matrix([[cos(q5), -sin(q5), 4, a4],
                   [sin(q5) * cos(alpha4), cos(q5) * cos(alpha4), -sin(alpha4), -sin(alpha4) * d5],
                   [sin(q5) * sin(alpha4), cos(q5) * sin(alpha4), cos(alpha4), cos(alpha4) * d5],
                   [0, 0, 0, 1]]).subs(s)

    T5_6 = Matrix([[cos(q6), -sin(q6), 5, a5],
                   [sin(q6) * cos(alpha5), cos(q6) * cos(alpha5), -sin(alpha5), -sin(alpha5) * d6],
                   [sin(q6) * sin(alpha5), cos(q6) * sin(alpha5), cos(alpha5), cos(alpha5) * d6],
                   [0, 0, 0, 1]]).subs(s)

    T6_G = Matrix([[cos(q7), -sin(q7), 6, a6],
                   [sin(q7) * cos(alpha6), cos(q7) * cos(alpha6), -sin(alpha6), -sin(alpha6) * d7],
                   [sin(q7) * sin(alpha6), cos(q7) * sin(alpha6), cos(alpha6), cos(alpha6) * d7],
                   [0, 0, 0, 1]]).subs(s)

    # Create individual transformation matrices
    T0_2 = simplify(T0_1 * T1_2)  # base_link to link 2
    T0_3 = simplify(T0_2 * T2_3)  # base_link to link 3
    T0_4 = simplify(T0_3 * T3_4)  # base_link to link 4
    T0_5 = simplify(T0_4 * T4_5)  # base_link to link 5
    T0_6 = simplify(T0_5 * T5_6)  # base_link to link 6
    T0_G = simplify(T0_6 * T6_G)  # base_link to link G

    # Correction for Gripper_link, URDF to DH
    # Rotate reference frame about z by 180deg and about y by -90deg
    R_z = Matrix([
        [cos(pi), -sin(pi), 0, 0],
        [sin(pi), cos(pi), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])
    R_y = Matrix([
        [cos(-pi / 2), 0, sin(-pi / 2), 0],
        [0, 1, 0, 0],
        [-sin(-pi / 2), 0, cos(-pi / 2), 0],
        [0, 0, 0, 1],
    ])
    R_corr = simplify(R_z * R_y)

    # total homogeneous transform from base_link to gripper_link
    T_total = simplify(T0_G * R_corr)

    # compare with tf_echo
    subs = {q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}
    for t in (T0_1, T0_2, T0_3, T0_4, T0_5, T0_6, T0_G, T_total,):
        print(t.evalf(subs=subs))

    # Extract rotation matrices from the transformation matrices
    R0_1 = T0_1[0:3, 0:3]
    R0_3 = T0_3[0:3, 0:3]
    R0_6 = T0_6[0:3, 0:3]

    return R_corr, R0_3, R0_6


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')  # theta_i
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        # Create Modified DH parameters
        s = {
            alpha0: 0, a0: 0, d1: 0.75,
            alpha1: -pi / 2, a1: 0.35, d2: 0, q2: q2 - pi / 2,
            alpha2: 0, a2: 1.25, d3: 0,
            alpha3: -pi / 2, a3: -0.054, d4: 1.50,
            alpha4: pi / 2, a4: 0, d5: 0,
            alpha5: -pi / 2, a5: 0, d6: 0,
            alpha6: 0, a6: 0, d7: 0.303, q7: 0,
        }

        # R_corr, R0_3, R0_6 = calculate_forward_kinematics()
        R_corr, R0_3, R0_6 = get_pre_calculated_transforms()

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                 req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here 
            # Compensate for rotation discrepancy between DH parameters and Gazebo
            dG = 0.303
            # WC_xyz = Matrix([px, py, pz]).T - R0_6 * Matrix([0, 0, dG]).T

            Rrpy = Rot('Z', yaw) * Rot('Y', pitch) * Rot('X', roll) * R_corr
            nx, ny, nz = Rrpy[0:3, 2]
            wx = px - dG * nx
            wy = py - dG * ny
            wz = pz - dG * nz

            # Calculate joint angles using Geometric IK method
            A = s[a2]
            B = s[d4]

            theta1 = atan2(wy, wx)

            C = sqrt(wz ** 2 + wy ** 2)
            if C > A + B:
                # no valid position possible
                print("invalid")
                continue

            theta3 = asin((wz ** 2 + wy ** 2 - A ** 2 - B ** 2) / 2 * A * B)
            beta = atan2(wz, wx)
            b = acos((wz ** 2 + wy ** 2 + A ** 2 - B ** 2) / 2 * A * C)

            theta2 = pi/2 - beta - b
            theta3 = C - pi/2

            # if theta3 < 0:
            #     theta2 = beta + b
            # else:
            #     theta2 = beta - b

            # B = sqrt(wz ** 2 + wy ** 2)
            # wc1 = acos(wy / b)
            # a = acos(a2 / b)
            # theta2 = 42
            # z3 = sin(theta2) * a2
            # x3 = cos(theta2) * a2
            # theta3 = sin((wz - z3) / d4)
            # # theta3 = cos((wx - x3)/d4)

            R3_6 = R0_3.inv('LU') * Rrpy
            R3_6 = R0_3.inv('LU') * R0_6

            subs = {q1: theta1, q2: theta2, q3: theta3, q4: 0, q5: 0, q6: 0}
            R3_6_num = R3_6.evalf(subs=subs)
            theta4, theta5, theta6 = R3_6_num[0, :]

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()


if __name__ == "__main__":
    IK_server()
