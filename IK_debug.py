from sympy import *
from time import time
from mpmath import radians
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.

Own test cases:
joint angles: adjust in joint_state_publisher
EE pos/orientation: gripper_link
WC pos: link_5

'''

test_cases = {1: [[[2.16135, -1.42635, 1.55109],
                   [0.708611, 0.186356, -0.157931, 0.661967]],
                  [1.89451, -1.44302, 1.69366],
                  [-0.65, 0.45, -0.36, 0.95, 0.79, 0.49]],
              2: [[[-0.56754, 0.93663, 3.0038],
                   [0.62073, 0.48318, 0.38759, 0.480629]],
                  [-0.638, 0.64198, 2.9988],
                  [-0.79, -0.11, -2.33, 1.94, 1.14, -3.68]],
              3: [[[-1.3863, 0.02074, 0.90986],
                   [0.01735, -0.2179, 0.9025, 0.371016]],
                  [-1.1669, -0.17989, 0.85137],
                  [-2.99, -0.12, 0.94, 4.06, 1.29, -4.12]],
              4: [[[1.8535, 0.80515, 0.93324],
                   [-0.27513, -0.095351, 0.5751, 0.76451]],
                  [1.7564, 0.52281, 0.98495],
                  [0.29, 0.27, 0.39, 4.98, -0.98, -5.97]],
              5: [[[-1.3681, -0.96189, 1.1533],
                   [0.22419, 0.076121, 0.90377, 0.35657]],
                  [-1.1726, -1.1675, 1.047],
                  [-2.36, 0.11, 0.53, 1.29, -1.80, 5.64]],
              0: [[[2.1529, 0, 1.9465],
                   [0, -0.00014835, 0, 1]],
                  [1.8499, 0, 1.9464],
                  [0, 0, 0, 0, 0, 0]],
              }


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


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0

    class Position:
        def __init__(self, EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]

    class Orientation:
        def __init__(self, EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self, position, orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position, orientation)

    class Pose:
        def __init__(self, comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()

    ########################################################################################
    ## 

    ## Insert IK code here!

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

    use_precomputed_transforms = False
    use_precomputed_transforms = True

    if use_precomputed_transforms:
        T0_5 = Matrix([[(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3), -(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), -sin(q1)*cos(q4) + sin(q4)*sin(q2 + q3)*cos(q1), (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*cos(q1)], [(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3), -(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3), sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4), (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*sin(q1)], [-sin(q5)*sin(q2 + q3) + cos(q4)*cos(q5)*cos(q2 + q3), -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5), sin(q4)*cos(q2 + q3), -1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75], [0, 0, 0, 1]])
        T_total = Matrix([[-(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), ((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) - (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6), ((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6), -0.303*(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*cos(q1) + 0.303*cos(q1)*cos(q5)*cos(q2 + q3)], [-(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3), ((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) + (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q6), ((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q6), -0.303*(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*sin(q1) + 0.303*sin(q1)*cos(q5)*cos(q2 + q3)], [-sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5), -(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) + sin(q4)*cos(q6)*cos(q2 + q3), -(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - sin(q4)*sin(q6)*cos(q2 + q3), -0.303*sin(q5)*cos(q4)*cos(q2 + q3) - 0.303*sin(q2 + q3)*cos(q5) - 1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75], [0, 0, 0, 1]])
        R_corr = Matrix([[0, 0, 1, 0], [0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1]])

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

    # Extract end-effector position and orientation from request
    # px,py,pz = end-effector position
    # roll, pitch, yaw = end-effector orientation
    px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z

    # (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
    (roll, pitch, yaw) = tf.euler_from_quaternion(
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
    A = s[a2]
    B = s[d4]

    theta1 = atan2(wy, wx)

    C = sqrt(wz ** 2 + wy ** 2)
    if C > A + B:
        # no valid position possible
        print("invalid")

    theta3 = asin((wz ** 2 + wy ** 2 - A ** 2 - B ** 2) / 2 * A * B)


    # beta = atan2(wz, wy)
    beta = atan2(wy, wz)
    b = acos((wz ** 2 + wy ** 2 + A ** 2 - B ** 2) / 2 * A * C)

    # print(beta)
    # print(b)

    theta2 = pi / 2 - beta - b
    theta2 = b + beta
    # theta3 = C - pi / 2

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

    #Matrix([[-0.997016358482613*(sin(q1)/(sin(q1)**2/cos(q1) + cos(q1)) + sin(q1)*cos(q2 + q3)**2/((sin(q1)**2/cos(q1) + cos(q1))*(-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3))*sin(q2 + q3)))/(sin(q2 + q3)*cos(q1)) + 0.0228997426720887*(-(-cos(q2 + q3)/(sin(q2 + q3)*cos(q1)) + sin(q1)**2*cos(q2 + q3)/((sin(q1)**2/cos(q1) + cos(q1))*sin(q2 + q3)*cos(q1)**2))*cos(q1)*cos(q2 + q3)/(-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3)) + 1 - sin(q1)**2/((sin(q1)**2/cos(q1) + cos(q1))*cos(q1)))/(sin(q2 + q3)*cos(q1)) + 0.0737155526576441*cos(q2 + q3)/((-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3))*sin(q2 + q3)), 0.0541428488017548*(sin(q1)/(sin(q1)**2/cos(q1) + cos(q1)) + sin(q1)*cos(q2 + q3)**2/((sin(q1)**2/cos(q1) + cos(q1))*(-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3))*sin(q2 + q3)))/(sin(q2 + q3)*cos(q1)) - 0.473197751193797*(-(-cos(q2 + q3)/(sin(q2 + q3)*cos(q1)) + sin(q1)**2*cos(q2 + q3)/((sin(q1)**2/cos(q1) + cos(q1))*sin(q2 + q3)*cos(q1)**2))*cos(q1)*cos(q2 + q3)/(-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3)) + 1 - sin(q1)**2/((sin(q1)**2/cos(q1) + cos(q1))*cos(q1)))/(sin(q2 + q3)*cos(q1)) + 0.879290873482014*cos(q2 + q3)/((-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3))*sin(q2 + q3)), 0.0550175684822593*(sin(q1)/(sin(q1)**2/cos(q1) + cos(q1)) + sin(q1)*cos(q2 + q3)**2/((sin(q1)**2/cos(q1) + cos(q1))*(-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3))*sin(q2 + q3)))/(sin(q2 + q3)*cos(q1)) + 0.880658554747914*(-(-cos(q2 + q3)/(sin(q2 + q3)*cos(q1)) + sin(q1)**2*cos(q2 + q3)/((sin(q1)**2/cos(q1) + cos(q1))*sin(q2 + q3)*cos(q1)**2))*cos(q1)*cos(q2 + q3)/(-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3)) + 1 - sin(q1)**2/((sin(q1)**2/cos(q1) + cos(q1))*cos(q1)))/(sin(q2 + q3)*cos(q1)) + 0.470546041432307*cos(q2 + q3)/((-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3))*sin(q2 + q3))], [0.0228997426720887*(-cos(q2 + q3)/(sin(q2 + q3)*cos(q1)) + sin(q1)**2*cos(q2 + q3)/((sin(q1)**2/cos(q1) + cos(q1))*sin(q2 + q3)*cos(q1)**2))/(-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3)) - 0.0737155526576441/(-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3)) + 0.997016358482613*sin(q1)*cos(q2 + q3)/((sin(q1)**2/cos(q1) + cos(q1))*(-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3))*sin(q2 + q3)*cos(q1)), -0.473197751193797*(-cos(q2 + q3)/(sin(q2 + q3)*cos(q1)) + sin(q1)**2*cos(q2 + q3)/((sin(q1)**2/cos(q1) + cos(q1))*sin(q2 + q3)*cos(q1)**2))/(-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3)) - 0.879290873482014/(-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3)) - 0.0541428488017548*sin(q1)*cos(q2 + q3)/((sin(q1)**2/cos(q1) + cos(q1))*(-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3))*sin(q2 + q3)*cos(q1)), 0.880658554747914*(-cos(q2 + q3)/(sin(q2 + q3)*cos(q1)) + sin(q1)**2*cos(q2 + q3)/((sin(q1)**2/cos(q1) + cos(q1))*sin(q2 + q3)*cos(q1)**2))/(-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3)) - 0.470546041432307/(-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3)) - 0.0550175684822593*sin(q1)*cos(q2 + q3)/((sin(q1)**2/cos(q1) + cos(q1))*(-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3))*sin(q2 + q3)*cos(q1))], [-0.0228997426720887*sin(q1)/((sin(q1)**2/cos(q1) + cos(q1))*cos(q1)) - 0.997016358482613/(sin(q1)**2/cos(q1) + cos(q1)), 0.473197751193797*sin(q1)/((sin(q1)**2/cos(q1) + cos(q1))*cos(q1)) + 0.0541428488017548/(sin(q1)**2/cos(q1) + cos(q1)), -0.880658554747914*sin(q1)/((sin(q1)**2/cos(q1) + cos(q1))*cos(q1)) + 0.0550175684822593/(sin(q1)**2/cos(q1) + cos(q1))]])
    #Matrix([[(((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) - (4*sin(q1)*sin(q4) - sin(q1)*cos(q4) + sin(q4)*sin(q2 + q3)*cos(q1) + 4*sin(q2 + q3)*cos(q1)*cos(q4))*sin(q6))*(-(-cos(q2 + q3)/(sin(q2 + q3)*cos(q1)) + sin(q1)**2*cos(q2 + q3)/((sin(q1)**2/cos(q1) + cos(q1))*sin(q2 + q3)*cos(q1)**2))*cos(q1)*cos(q2 + q3)/(-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3)) + 1 - sin(q1)**2/((sin(q1)**2/cos(q1) + cos(q1))*cos(q1)))/(sin(q2 + q3)*cos(q1)) + (((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + 4*sin(q1)*sin(q2 + q3)*cos(q4) - 4*sin(q4)*cos(q1) + cos(q1)*cos(q4))*sin(q6))*(sin(q1)/(sin(q1)**2/cos(q1) + cos(q1)) + sin(q1)*cos(q2 + q3)**2/((sin(q1)**2/cos(q1) + cos(q1))*(-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3))*sin(q2 + q3)))/(sin(q2 + q3)*cos(q1)) - ((-sin(q5)*sin(q2 + q3) + cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - (sin(q4) + 4*cos(q4))*sin(q6)*cos(q2 + q3))*cos(q2 + q3)/((-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3))*sin(q2 + q3)), ((-(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) - sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) - (4*sin(q1)*sin(q4) - sin(q1)*cos(q4) + sin(q4)*sin(q2 + q3)*cos(q1) + 4*sin(q2 + q3)*cos(q1)*cos(q4))*cos(q6))*(-(-cos(q2 + q3)/(sin(q2 + q3)*cos(q1)) + sin(q1)**2*cos(q2 + q3)/((sin(q1)**2/cos(q1) + cos(q1))*sin(q2 + q3)*cos(q1)**2))*cos(q1)*cos(q2 + q3)/(-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3)) + 1 - sin(q1)**2/((sin(q1)**2/cos(q1) + cos(q1))*cos(q1)))/(sin(q2 + q3)*cos(q1)) + ((-(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) - sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + 4*sin(q1)*sin(q2 + q3)*cos(q4) - 4*sin(q4)*cos(q1) + cos(q1)*cos(q4))*cos(q6))*(sin(q1)/(sin(q1)**2/cos(q1) + cos(q1)) + sin(q1)*cos(q2 + q3)**2/((sin(q1)**2/cos(q1) + cos(q1))*(-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3))*sin(q2 + q3)))/(sin(q2 + q3)*cos(q1)) - ((sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) - (sin(q4) + 4*cos(q4))*cos(q6)*cos(q2 + q3))*cos(q2 + q3)/((-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3))*sin(q2 + q3)), (sin(q1)/(sin(q1)**2/cos(q1) + cos(q1)) + sin(q1)*cos(q2 + q3)**2/((sin(q1)**2/cos(q1) + cos(q1))*(-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3))*sin(q2 + q3)))*((-sin(q1)*sin(q2 + q3)*cos(q4) + sin(q4)*cos(q1))*sin(q5) + (5*sin(q1)*sin(q2 + q3)*cos(q4) - 5*sin(q4)*cos(q1))*cos(q5) + 5*sin(q1)*sin(q5)*cos(q2 + q3) + sin(q1)*cos(q5)*cos(q2 + q3))/(sin(q2 + q3)*cos(q1)) + (-(-cos(q2 + q3)/(sin(q2 + q3)*cos(q1)) + sin(q1)**2*cos(q2 + q3)/((sin(q1)**2/cos(q1) + cos(q1))*sin(q2 + q3)*cos(q1)**2))*cos(q1)*cos(q2 + q3)/(-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3)) + 1 - sin(q1)**2/((sin(q1)**2/cos(q1) + cos(q1))*cos(q1)))*((-sin(q1)*sin(q4) - sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + (5*sin(q1)*sin(q4) + 5*sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + 5*sin(q5)*cos(q1)*cos(q2 + q3) + cos(q1)*cos(q5)*cos(q2 + q3))/(sin(q2 + q3)*cos(q1)) - (-5*sin(q5)*sin(q2 + q3) - sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5) + 5*cos(q4)*cos(q5)*cos(q2 + q3))*cos(q2 + q3)/((-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3))*sin(q2 + q3))], [(((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) - (4*sin(q1)*sin(q4) - sin(q1)*cos(q4) + sin(q4)*sin(q2 + q3)*cos(q1) + 4*sin(q2 + q3)*cos(q1)*cos(q4))*sin(q6))*(-cos(q2 + q3)/(sin(q2 + q3)*cos(q1)) + sin(q1)**2*cos(q2 + q3)/((sin(q1)**2/cos(q1) + cos(q1))*sin(q2 + q3)*cos(q1)**2))/(-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3)) - (((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + 4*sin(q1)*sin(q2 + q3)*cos(q4) - 4*sin(q4)*cos(q1) + cos(q1)*cos(q4))*sin(q6))*sin(q1)*cos(q2 + q3)/((sin(q1)**2/cos(q1) + cos(q1))*(-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3))*sin(q2 + q3)*cos(q1)) + ((-sin(q5)*sin(q2 + q3) + cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - (sin(q4) + 4*cos(q4))*sin(q6)*cos(q2 + q3))/(-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3)), ((-(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) - sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) - (4*sin(q1)*sin(q4) - sin(q1)*cos(q4) + sin(q4)*sin(q2 + q3)*cos(q1) + 4*sin(q2 + q3)*cos(q1)*cos(q4))*cos(q6))*(-cos(q2 + q3)/(sin(q2 + q3)*cos(q1)) + sin(q1)**2*cos(q2 + q3)/((sin(q1)**2/cos(q1) + cos(q1))*sin(q2 + q3)*cos(q1)**2))/(-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3)) - ((-(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) - sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + 4*sin(q1)*sin(q2 + q3)*cos(q4) - 4*sin(q4)*cos(q1) + cos(q1)*cos(q4))*cos(q6))*sin(q1)*cos(q2 + q3)/((sin(q1)**2/cos(q1) + cos(q1))*(-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3))*sin(q2 + q3)*cos(q1)) + ((sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) - (sin(q4) + 4*cos(q4))*cos(q6)*cos(q2 + q3))/(-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3)), (-cos(q2 + q3)/(sin(q2 + q3)*cos(q1)) + sin(q1)**2*cos(q2 + q3)/((sin(q1)**2/cos(q1) + cos(q1))*sin(q2 + q3)*cos(q1)**2))*((-sin(q1)*sin(q4) - sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + (5*sin(q1)*sin(q4) + 5*sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + 5*sin(q5)*cos(q1)*cos(q2 + q3) + cos(q1)*cos(q5)*cos(q2 + q3))/(-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3)) + (-5*sin(q5)*sin(q2 + q3) - sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5) + 5*cos(q4)*cos(q5)*cos(q2 + q3))/(-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3)) - ((-sin(q1)*sin(q2 + q3)*cos(q4) + sin(q4)*cos(q1))*sin(q5) + (5*sin(q1)*sin(q2 + q3)*cos(q4) - 5*sin(q4)*cos(q1))*cos(q5) + 5*sin(q1)*sin(q5)*cos(q2 + q3) + sin(q1)*cos(q5)*cos(q2 + q3))*sin(q1)*cos(q2 + q3)/((sin(q1)**2/cos(q1) + cos(q1))*(-sin(q2 + q3) - cos(q2 + q3)**2/sin(q2 + q3))*sin(q2 + q3)*cos(q1))], [-(((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) - (4*sin(q1)*sin(q4) - sin(q1)*cos(q4) + sin(q4)*sin(q2 + q3)*cos(q1) + 4*sin(q2 + q3)*cos(q1)*cos(q4))*sin(q6))*sin(q1)/((sin(q1)**2/cos(q1) + cos(q1))*cos(q1)) + (((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + 4*sin(q1)*sin(q2 + q3)*cos(q4) - 4*sin(q4)*cos(q1) + cos(q1)*cos(q4))*sin(q6))/(sin(q1)**2/cos(q1) + cos(q1)), -((-(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) - sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) - (4*sin(q1)*sin(q4) - sin(q1)*cos(q4) + sin(q4)*sin(q2 + q3)*cos(q1) + 4*sin(q2 + q3)*cos(q1)*cos(q4))*cos(q6))*sin(q1)/((sin(q1)**2/cos(q1) + cos(q1))*cos(q1)) + ((-(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) - sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + 4*sin(q1)*sin(q2 + q3)*cos(q4) - 4*sin(q4)*cos(q1) + cos(q1)*cos(q4))*cos(q6))/(sin(q1)**2/cos(q1) + cos(q1)), -((-sin(q1)*sin(q4) - sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + (5*sin(q1)*sin(q4) + 5*sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + 5*sin(q5)*cos(q1)*cos(q2 + q3) + cos(q1)*cos(q5)*cos(q2 + q3))*sin(q1)/((sin(q1)**2/cos(q1) + cos(q1))*cos(q1)) + ((-sin(q1)*sin(q2 + q3)*cos(q4) + sin(q4)*cos(q1))*sin(q5) + (5*sin(q1)*sin(q2 + q3)*cos(q4) - 5*sin(q4)*cos(q1))*cos(q5) + 5*sin(q1)*sin(q5)*cos(q2 + q3) + sin(q1)*cos(q5)*cos(q2 + q3))/(sin(q1)**2/cos(q1) + cos(q1))]])

    # Rrpy = Rrpy[:3,:3]
    # R3_6rhs = R0_3.inv('LU') * Rrpy
    # R3_6lhs = R0_3.inv('LU') * R0_6
    #
    # subs = {q1: theta1, q2: theta2, q3: theta3}
    # R3_6rhs_n = R3_6rhs.evalf(subs=subs)
    # R3_6lhs_n = simplify(R3_6lhs.evalf(subs=subs))
    #
    # theta4, theta5, theta6 = R3_6rhs_n[0, :]


    theta4, theta5, theta6 = 0,0,0


    # theta1 = 0
    # theta2 = 0
    # theta3 = 0
    # theta4 = 0
    # theta5 = 0
    # theta6 = 0

    for i, t in enumerate([theta1, theta2, theta3, theta4, theta5, theta6]):
        print("theta_%s = %s" % (i+1, t))

    ## 
    ########################################################################################

    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!


    # compare with tf_echo
    # test: links all zero
    # subs = {q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}
    # gripper_link_pos = [2.153, 0.000, 1.947]

    subs = {q1: test_case[2][0], q2: test_case[2][1], q3: test_case[2][2], q4: test_case[2][3], q5: test_case[2][4],
            q6: test_case[2][5]}
    # i=1
    # for t in (T0_1, T0_2, T0_3, T0_4, T0_5, T0_6, T0_G, T_total,):
    #     print("T0_%s" % i)
    #     pprint(t.evalf(subs=subs))
    #     pprint(t.evalf(subs=subs) * Matrix([0,0,0,1]))
    #     i+=1

    # Extract rotation matrices from the transformation matrices
    # R0_1 = T0_1[0:3, 0:3]
    # R0_3 = T0_3[0:3, 0:3]
    # R0_6 = T0_6[0:3, 0:3]

    # evaluate with given angles
    # subs = {q1: test_case[2][0], q2: test_case[2][1], q3: test_case[2][2], q4: test_case[2][3], q5: test_case[2][4],
    #         q6: test_case[2][5]}

    # evaluate with angles from inverse kinematics
    subs = {q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6}

    # WC
    T0_5_num = T0_5.evalf(subs=subs)
    # EE
    T_total_num = T_total.evalf(subs=subs)

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    # your_wc = T0_5_num[:3, 3]  # <--- Load your calculated WC values in this array
    your_wc = [wx, wy, wz]  # <--- Load your calculated WC values in this array
    # your_ee = [1, 1, 1]  # <--- Load your calculated end effector value from your forward kinematics
    your_ee = T_total_num[:3, 3]  # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time() - start_time))

    # Find WC error
    if not (sum(your_wc) == 3):
        wc_x_e = abs(your_wc[0] - test_case[1][0])
        wc_y_e = abs(your_wc[1] - test_case[1][1])
        wc_z_e = abs(your_wc[2] - test_case[1][2])
        wc_offset = sqrt(wc_x_e ** 2 + wc_y_e ** 2 + wc_z_e ** 2)
        if wc_offset > 0.01:
            print ("\nWrist error for x position is: %04.8f" % wc_x_e)
            print ("Wrist error for y position is: %04.8f" % wc_y_e)
            print ("Wrist error for z position is: %04.8f" % wc_z_e)
            print ("Overall wrist offset is: %04.8f units" % wc_offset)
        else:
            print("WC OK, at %s" % your_wc)

    # # Find theta errors
    t_1_e = abs(theta1 - test_case[2][0])
    t_2_e = abs(theta2 - test_case[2][1])
    t_3_e = abs(theta3 - test_case[2][2])
    t_4_e = abs(theta4 - test_case[2][3])
    t_5_e = abs(theta5 - test_case[2][4])
    t_6_e = abs(theta6 - test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not (sum(your_ee) == 3):
        ee_x_e = abs(your_ee[0] - test_case[0][0][0])
        ee_y_e = abs(your_ee[1] - test_case[0][0][1])
        ee_z_e = abs(your_ee[2] - test_case[0][0][2])
        ee_offset = sqrt(ee_x_e ** 2 + ee_y_e ** 2 + ee_z_e ** 2)
        if ee_offset > 0.01:
            print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
            print ("End effector error for y position is: %04.8f" % ee_y_e)
            print ("End effector error for z position is: %04.8f" % ee_z_e)
            print ("Overall end effector offset is: %04.8f units \n" % ee_offset)
        else:
            print("EE OK, at %s" % your_ee)


if __name__ == "__main__":
    # Change test case number for different scenarios
    for test_case_number in range(len(test_cases)):
        test_code(test_cases[test_case_number])
        break
