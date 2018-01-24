## Project: Kinematics Pick & Place

---


[//]: # (Image References)

[reference_frames]: writeup/reference_frames2.jpg
[inverse_position]: writeup/inverse_position2.jpg
[inverse_orientation]: writeup/inverse_orientation2.jpg

---

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

A drawing of the joint reference frames and DH parameters is given below.

![alt text][reference_frames]

The values for the "a" and "d" can be extracted from the kr210.urdf.xacro file, e.g.

        <joint name="joint_1" type="revolute">
            <origin xyz="0 0 0.33" rpy="0 0 0"/>
            ...
          <joint name="joint_2" type="revolute">
            <origin xyz="0.35 0 0.42" rpy="0 0 0"/>

-> "d1" = 0.33 + 0.42

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.33+0.42 | qi
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | 0
3->4 |  - pi/2 | -0.054 | 0.96+0.54 | 0
4->5 | pi/2 | 0 | 0 | 0
5->6 | -pi/2 | 0 | 0 | 0
6->EE | 0 | 0 | 0.193+0.11 | 0

An example for the homogeneous transform between Joint 0 (base_link) and Joint 1 is given below.

        #### Homogeneous Transforms
        T0_1 = Matrix([[cos(q1), -sin(q1), 0, a0],
                       [sin(q1) * cos(alpha0), cos(q1) * cos(alpha0), -sin(alpha0), -sin(alpha0) * d1],
                       [sin(q1) * sin(alpha0), cos(q1) * sin(alpha0), cos(alpha0), cos(alpha0) * d1],
                       [0, 0, 0, 1]])
        T0_1 = T0_1.subs(s)

The transforms between the other joint are analogous, using the respective parameters from the DH table.

The transform between the base_link and the gripper_link can be obtained by chaining the individual transforms between intermediate joints together, i.e.:

        # Create individual transformation matrices
        T0_2 = simplify(T0_1 * T1_2)  # base_link to link 2
        T0_3 = simplify(T0_2 * T2_3)  # base_link to link 3
        T0_4 = simplify(T0_3 * T3_4)  # base_link to link 4
        T0_5 = simplify(T0_4 * T4_5)  # base_link to link 5
        T0_6 = simplify(T0_5 * T5_6)  # base_link to link 6
        T0_G = simplify(T0_6 * T6_G)  # base_link to link G


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

First we obtain the position of the wrist center (global reference frame).

    # px, py, pz and roll, pitch, yaw are given independent parameters

    # from the DH table:
    d6 = 0
    ee_length = 0.303

    # to obtain the rotation matrix we apply the given roll, pitch and yaw and
    # then correct to account for the difference between the gazebo reference frame
    # and our DH reference frame. (R_corr = Rotate reference frame about z by 180deg and about y by -90deg)
    Rrpy = Rot('Z', yaw) * Rot('Y', pitch) * Rot('X', roll) * R_corr
    nx, ny, nz = Rrpy[:3, 2]

    # wrist center is found by moving back for the length of the End-Effector along the End-Effector orientation vector.
    wx = px - (d6 + ee_length) * nx
    wy = py - (d6 + ee_length) * ny
    wz = pz - (d6 + ee_length) * nz

Now we can compute the angles necessary to obtain the given position of the wrist center (theta{1,2,3}).

![alt text][inverse_position]

The angles of the wrist (theta{4,5,6}) to obtain the given orientation of the end-effector are acquired by solving the inverse orientation problem.
The rotation matrix for joint3 -> joint 6 can be computed by recognizing that the rotation matrix for joint0 -> joint 6 (from forward kinematics, see above)
is equal to the rotation matrix Rrpy we computed from the given independent parameters. By using the inverse of R0_3 (also from forward kinematics),
we find R3_6, which is only dependent on theta{4,5,6}. R3_6_num contains the numeric values, thus we can solve for the thetas.

    R3_6_num = R0_3_num.inv('LU') * Rrpy3x3
    R0_6 = T0_6[:3, :3]
    R3_6 = R0_3.inv('LU') * R0_6
    R3_6 = simplify(R3_6.evalf(subs=subs))

![alt text][inverse_orientation]



### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  

##### 1. Precomputed transformation matrices
The calculation of the forward kinematics transformations (especially the simplify call) were found to be to slow for productive use (around 40 seconds per call).
Since the transformation matrices don't change as long as the serial manipulator design is the same, they only need to be computed once an can then
be reused.

    if use_precomputed_transforms:
        # precomputed transforms are stored here
        ...
    else:
        # compute from scratch

