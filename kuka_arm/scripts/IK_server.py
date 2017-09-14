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

    else:
        raise ValueError

    return R


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

        use_precomputed_transforms = True

        if use_precomputed_transforms:
            T0_5 = Matrix([[(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3), -(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), -sin(q1)*cos(q4) + sin(q4)*sin(q2 + q3)*cos(q1), (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*cos(q1)], [(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3), -(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3), sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4), (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*sin(q1)], [-sin(q5)*sin(q2 + q3) + cos(q4)*cos(q5)*cos(q2 + q3), -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5), sin(q4)*cos(q2 + q3), -1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75], [0, 0, 0, 1]])
            T_total = Matrix([[-(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), ((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) - (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6), ((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6), -0.303*(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*cos(q1) + 0.303*cos(q1)*cos(q5)*cos(q2 + q3)], [-(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3), ((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) + (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q6), ((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q6), -0.303*(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*sin(q1) + 0.303*sin(q1)*cos(q5)*cos(q2 + q3)], [-sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5), -(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) + sin(q4)*cos(q6)*cos(q2 + q3), -(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - sin(q4)*sin(q6)*cos(q2 + q3), -0.303*sin(q5)*cos(q4)*cos(q2 + q3) - 0.303*sin(q2 + q3)*cos(q5) - 1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75], [0, 0, 0, 1]])
            R_corr = Matrix([[0, 0, 1, 0], [0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1]])
            T0_3 = Matrix([[sin(q2 + q3)*cos(q1), cos(q1)*cos(q2 + q3), -sin(q1), (1.25*sin(q2) + 0.35)*cos(q1)], [sin(q1)*sin(q2 + q3), sin(q1)*cos(q2 + q3), cos(q1), (1.25*sin(q2) + 0.35)*sin(q1)], [cos(q2 + q3), -sin(q2 + q3), 0, 1.25*cos(q2) + 0.75], [0, 0, 0, 1]])
            T0_6 = Matrix([[((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6), -((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6), -(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*cos(q1)], [((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q6), -((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q6), -(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3), (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*sin(q1)], [-(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - sin(q4)*sin(q6)*cos(q2 + q3), (sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) - sin(q4)*cos(q6)*cos(q2 + q3), -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5), -1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75], [0, 0, 0, 1]])
            R3_6 = Matrix([[-1.0*sin(q4)*sin(q6) + 1.0*cos(q4)*cos(q5)*cos(q6), -1.0*sin(q4)*cos(q6) - 1.0*sin(q6)*cos(q4)*cos(q5), -1.0*sin(q5)*cos(q4)], [sin(q5)*cos(q6), -sin(q5)*sin(q6), cos(q5)], [-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4), sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6), sin(q4)*sin(q5)]])

        else:
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

            T4_5 = Matrix([[cos(q5), -sin(q5), 0, a4],
                           [sin(q5) * cos(alpha4), cos(q5) * cos(alpha4), -sin(alpha4), -sin(alpha4) * d5],
                           [sin(q5) * sin(alpha4), cos(q5) * sin(alpha4), cos(alpha4), cos(alpha4) * d5],
                           [0, 0, 0, 1]]).subs(s)

            T5_6 = Matrix([[cos(q6), -sin(q6), 0, a5],
                           [sin(q6) * cos(alpha5), cos(q6) * cos(alpha5), -sin(alpha5), -sin(alpha5) * d6],
                           [sin(q6) * sin(alpha5), cos(q6) * sin(alpha5), cos(alpha5), cos(alpha5) * d6],
                           [0, 0, 0, 1]]).subs(s)

            T6_G = Matrix([[cos(q7), -sin(q7), 0, a6],
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

        # Inverse Kinematics
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            print("Inverting pose %s of %s" % (x, len(req.poses)))
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

            # Compensate for rotation discrepancy between DH parameters and Gazebo
            d6 = 0
            ee_length = 0.303
            Rrpy = Rot('Z', yaw) * Rot('Y', pitch) * Rot('X', roll) * R_corr
            nx, ny, nz = Rrpy[:3, 2]
            wx = px - (d6 + ee_length) * nx
            wy = py - (d6 + ee_length) * ny
            wz = pz - (d6 + ee_length) * nz

            # Calculate joint angles using Geometric IK method
            theta1 = atan2(wy, wx)

            # link2 origin as offset
            pz = wz - 0.75
            # project wx, wy into z-y-plane
            px = sqrt(wx ** 2 + wy ** 2) - 0.35
            l2 = 1.25
            l3 = 1.50
            max_length_of_arms = l2 + l3
            beta = atan2(pz, px)
            lp = sqrt(pz ** 2 + px ** 2)

            if lp == max_length_of_arms:
                # arms must be fully extended
                theta3 = pi / 2
                theta2 = pi / 2 - beta
                print("Arm fully extended")

            elif lp > max_length_of_arms:
                # point is out of reach
                theta3 = 0
                theta2 = 0
                print("WC coordinates are out of reach: %s %s %s" % (wx, wy, wz))

            else:
                # c3 = (px**2 + pz**2 - l2**2 - l3**2)/(2*l2*l3)
                # s3 = sqrt(1.0 - c3**2)
                # s3_alt = - s3
                # a = atan2(s3, c3)
                # a_alt = atan2(s3_alt, c3)
                #
                # b = atan2(l3 * sin(a), l2 + l3 * cos(a))
                # b_alt = atan2(l3 * sin(a_alt), l2 + l3 * cos(a_alt))
                #
                # theta2 = pi/2 - beta - b
                # theta2_alt = pi/2 - beta - b_alt
                #
                # theta3 = pi / 2 - a
                # theta3_alt = pi / 2 - a_alt

                WC = [wx, wy, wz]
                side_a = 1.501  # d4
                side_b = sqrt((sqrt(wx ** 2 + wy ** 2) - 0.35) ** 2 + (wz - 0.75) ** 2)
                side_c = 1.25  # a2

                angle_a = acos((side_b ** 2 + side_c ** 2 - side_a ** 2) / (2 * side_b * side_c))
                angle_b = acos((side_a ** 2 + side_c ** 2 - side_b ** 2) / (2 * side_a * side_c))
                angle_c = acos((side_a ** 2 + side_b ** 2 - side_c ** 2) / (2 * side_a * side_b))

                theta2 = pi / 2 - angle_a - atan2(wz - 0.75, sqrt(wx ** 2 + wy ** 2) - 0.35)
                theta3 = pi / 2 - (angle_b + 0.036)  # 0.036 accounts for sag in link4 of -0.054

            subs = {q1: theta1, q2: theta2, q3: theta3}

            R0_3 = T0_3[:3, :3]
            R0_3_num = R0_3.evalf(subs=subs)
            Rrpy3x3 = Rrpy[:3, :3]

            R3_6_num = R0_3_num.inv('LU') * Rrpy3x3
            if not use_precomputed_transforms:
                R0_6 = T0_6[:3, :3]
                R3_6 = R0_3.inv('LU') * R0_6
                R3_6 = simplify(R3_6.evalf(subs=subs))

            theta4 = atan2(R3_6_num[2, 2], - R3_6_num[0, 2])
            theta5 = atan2(sqrt(R3_6_num[1, 0] ** 2 + (-R3_6_num[1, 1]) ** 2), R3_6_num[1, 2])
            theta6 = atan2(- R3_6_num[1, 1], R3_6_num[1, 0])

            # evaluate with angles from inverse kinematics
            subs = {q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6}
            # EE
            T_total_num = T_total.evalf(subs=subs)

            your_ee = T_total_num[:3, 3]  # <--- Load your calculated end effector value from your forward kinematics
            ee_x_e = abs(your_ee[0] - px)
            ee_y_e = abs(your_ee[1] - py)
            ee_z_e = abs(your_ee[2] - pz)
            ee_offset = sqrt(ee_x_e ** 2 + ee_y_e ** 2 + ee_z_e ** 2)
            if ee_offset > 0.01:
                print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
                print ("End effector error for y position is: %04.8f" % ee_y_e)
                print ("End effector error for z position is: %04.8f" % ee_z_e)
                print ("Overall end effector offset is: %04.8f units \n" % ee_offset)
            else:
                print("EE OK, at %s" % your_ee)

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
