## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[reference_frames]: reference_frames.png
[inverse_position]: inverse_position.png
[inverse_orientation]: inverse_orientation.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

A drawing of the joint reference frames and DH parameters is given below.
![alt text][reference_frames]



Here is an example of how to include an image in your writeup.

Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.
	

Your writeup should contain a DH parameter table with proper notations and description about how you obtained the table. Make sure to use the modified DH parameters discussed in this lesson. Please add an annotated figure of the robot with proper link assignments and joint rotations (Example figure provided in the writeup template). It is strongly recommended that you use pen and paper to create this figure to get a better understanding of the robot kinematics.

Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.
	

Your writeup should contain individual transform matrices about each joint using the DH table and a homogeneous transform matrix from base_link to gripper_link using only the position and orientation of the gripper_link. These matrices can be created using any software of your choice or hand written. Also include an explanation on how you created these matrices.

Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.
	

Based on the geometric Inverse Kinematics method described here, breakdown the IK problem into Position and Orientation problems. Derive the equations for individual joint angles. Your writeup must contain details about the steps you took to arrive at those equations. Add figures where necessary. If any given joint has multiple solutions, select the best solution and provide explanation about your choice (Hint: Observe the active robot workspace in this project and the fact that some joints have physical limits).




![alt text][image1]

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


T0_1
⎡cos(q₁)  -sin(q₁)  0   0  ⎤
⎢                          ⎥
⎢sin(q₁)  cos(q₁)   0   0  ⎥
⎢                          ⎥
⎢   0        0      1  0.75⎥
⎢                          ⎥
⎣   0        0      0   1  ⎦
T0_2
⎡sin(q₂)⋅cos(q₁)  cos(q₁)⋅cos(q₂)  -sin(q₁)  0.35⋅cos(q₁)⎤
⎢                                                        ⎥
⎢sin(q₁)⋅sin(q₂)  sin(q₁)⋅cos(q₂)  cos(q₁)   0.35⋅sin(q₁)⎥
⎢                                                        ⎥
⎢    cos(q₂)         -sin(q₂)         0          0.75    ⎥
⎢                                                        ⎥
⎣       0                0            0           1      ⎦
T0_3
⎡sin(q₂ + q₃)⋅cos(q₁)  cos(q₁)⋅cos(q₂ + q₃)  -sin(q₁)  (1.25⋅sin(q₂) + 0.35)⋅cos(q₁)⎤
⎢                                                                                   ⎥
⎢sin(q₁)⋅sin(q₂ + q₃)  sin(q₁)⋅cos(q₂ + q₃)  cos(q₁)   (1.25⋅sin(q₂) + 0.35)⋅sin(q₁)⎥
⎢                                                                                   ⎥
⎢    cos(q₂ + q₃)         -sin(q₂ + q₃)         0           1.25⋅cos(q₂) + 0.75     ⎥
⎢                                                                                   ⎥
⎣         0                     0               0                    1              ⎦
T0_4
⎡sin(q₁)⋅sin(q₄) + sin(q₂ + q₃)⋅cos(q₁)⋅cos(q₄)  sin(q₁)⋅cos(q₄) - sin(q₄)⋅sin(q₂ + q₃)⋅cos(q₁)   cos(q₁)⋅cos(q₂ + q₃)  (1.25⋅sin(q₂) - 0.054⋅sin(q₂ + q₃) + 1.5⋅cos(q₂ + q₃) + 0.35)⋅cos(q₁)⎤
⎢                                                                                                                                                                                            ⎥
⎢sin(q₁)⋅sin(q₂ + q₃)⋅cos(q₄) - sin(q₄)⋅cos(q₁)  -sin(q₁)⋅sin(q₄)⋅sin(q₂ + q₃) - cos(q₁)⋅cos(q₄)  sin(q₁)⋅cos(q₂ + q₃)  (1.25⋅sin(q₂) - 0.054⋅sin(q₂ + q₃) + 1.5⋅cos(q₂ + q₃) + 0.35)⋅sin(q₁)⎥
⎢                                                                                                                                                                                            ⎥
⎢             cos(q₄)⋅cos(q₂ + q₃)                            -sin(q₄)⋅cos(q₂ + q₃)                  -sin(q₂ + q₃)          -1.5⋅sin(q₂ + q₃) + 1.25⋅cos(q₂) - 0.054⋅cos(q₂ + q₃) + 0.75     ⎥
⎢                                                                                                                                                                                            ⎥
⎣                      0                                                0                                  0                                              1                                  ⎦
T0_5
⎡(sin(q₁)⋅sin(q₄) + sin(q₂ + q₃)⋅cos(q₁)⋅cos(q₄))⋅cos(q₅) + sin(q₅)⋅cos(q₁)⋅cos(q₂ + q₃)  -(sin(q₁)⋅sin(q₄) + sin(q₂ + q₃)⋅cos(q₁)⋅cos(q₄))⋅sin(q₅) + cos(q₁)⋅cos(q₅)⋅cos(q₂ + q₃)  -sin(q₁)⋅cos(q₄) + sin(q₄)⋅sin(q₂ + q₃)⋅cos(q₁)  (1.25⋅sin(q₂) - 0.054⋅sin(q₂ + q₃) + 1.5⋅cos(q₂ + q₃) + 0.35)⋅cos(q₁)⎤
⎢                                                                                                                                                                                                                                                                                                         ⎥
⎢(sin(q₁)⋅sin(q₂ + q₃)⋅cos(q₄) - sin(q₄)⋅cos(q₁))⋅cos(q₅) + sin(q₁)⋅sin(q₅)⋅cos(q₂ + q₃)  -(sin(q₁)⋅sin(q₂ + q₃)⋅cos(q₄) - sin(q₄)⋅cos(q₁))⋅sin(q₅) + sin(q₁)⋅cos(q₅)⋅cos(q₂ + q₃)  sin(q₁)⋅sin(q₄)⋅sin(q₂ + q₃) + cos(q₁)⋅cos(q₄)   (1.25⋅sin(q₂) - 0.054⋅sin(q₂ + q₃) + 1.5⋅cos(q₂ + q₃) + 0.35)⋅sin(q₁)⎥
⎢                                                                                                                                                                                                                                                                                                         ⎥
⎢                 -sin(q₅)⋅sin(q₂ + q₃) + cos(q₄)⋅cos(q₅)⋅cos(q₂ + q₃)                                      -sin(q₅)⋅cos(q₄)⋅cos(q₂ + q₃) - sin(q₂ + q₃)⋅cos(q₅)                                 sin(q₄)⋅cos(q₂ + q₃)                    -1.5⋅sin(q₂ + q₃) + 1.25⋅cos(q₂) - 0.054⋅cos(q₂ + q₃) + 0.75     ⎥
⎢                                                                                                                                                                                                                                                                                                         ⎥
⎣                                           0                                                                                        0                                                                     0                                                           1                                  ⎦
T0_6
⎡((sin(q₁)⋅sin(q₄) + sin(q₂ + q₃)⋅cos(q₁)⋅cos(q₄))⋅cos(q₅) + sin(q₅)⋅cos(q₁)⋅cos(q₂ + q₃))⋅cos(q₆) + (sin(q₁)⋅cos(q₄) - sin(q₄)⋅sin(q₂ + q₃)⋅cos(q₁))⋅sin(q₆)  -((sin(q₁)⋅sin(q₄) + sin(q₂ + q₃)⋅cos(q₁)⋅cos(q₄))⋅cos(q₅) + sin(q₅)⋅cos(q₁)⋅cos(q₂ + q₃))⋅sin(q₆) + (sin(q₁)⋅cos(q₄) - sin(q₄)⋅sin(q₂ + q₃)⋅cos(q₁))⋅cos(q₆)  -(sin(q₁)⋅sin(q₄) + sin(q₂ + q₃)⋅cos(q₁)⋅cos(q₄))⋅sin(q₅) + cos(q₁)⋅cos(q₅)⋅cos(q₂ + q₃)  (1.25⋅sin(q₂) - 0.054⋅sin(q₂ + q₃) + 1.5⋅cos(q₂ + q₃) + 0.35)⋅cos(q₁)⎤
⎢                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            ⎥
⎢((sin(q₁)⋅sin(q₂ + q₃)⋅cos(q₄) - sin(q₄)⋅cos(q₁))⋅cos(q₅) + sin(q₁)⋅sin(q₅)⋅cos(q₂ + q₃))⋅cos(q₆) - (sin(q₁)⋅sin(q₄)⋅sin(q₂ + q₃) + cos(q₁)⋅cos(q₄))⋅sin(q₆)  -((sin(q₁)⋅sin(q₂ + q₃)⋅cos(q₄) - sin(q₄)⋅cos(q₁))⋅cos(q₅) + sin(q₁)⋅sin(q₅)⋅cos(q₂ + q₃))⋅sin(q₆) - (sin(q₁)⋅sin(q₄)⋅sin(q₂ + q₃) + cos(q₁)⋅cos(q₄))⋅cos(q₆)  -(sin(q₁)⋅sin(q₂ + q₃)⋅cos(q₄) - sin(q₄)⋅cos(q₁))⋅sin(q₅) + sin(q₁)⋅cos(q₅)⋅cos(q₂ + q₃)  (1.25⋅sin(q₂) - 0.054⋅sin(q₂ + q₃) + 1.5⋅cos(q₂ + q₃) + 0.35)⋅sin(q₁)⎥
⎢                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            ⎥
⎢                               -(sin(q₅)⋅sin(q₂ + q₃) - cos(q₄)⋅cos(q₅)⋅cos(q₂ + q₃))⋅cos(q₆) - sin(q₄)⋅sin(q₆)⋅cos(q₂ + q₃)                                                                  (sin(q₅)⋅sin(q₂ + q₃) - cos(q₄)⋅cos(q₅)⋅cos(q₂ + q₃))⋅sin(q₆) - sin(q₄)⋅cos(q₆)⋅cos(q₂ + q₃)                                                     -sin(q₅)⋅cos(q₄)⋅cos(q₂ + q₃) - sin(q₂ + q₃)⋅cos(q₅)                        -1.5⋅sin(q₂ + q₃) + 1.25⋅cos(q₂) - 0.054⋅cos(q₂ + q₃) + 0.75     ⎥
⎢                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            ⎥
⎣                                                                             0                                                                                                                                                              0                                                                                                                           0                                                                                1                                  ⎦
T0_7
⎡((sin(q₁)⋅sin(q₄) + sin(q₂ + q₃)⋅cos(q₁)⋅cos(q₄))⋅cos(q₅) + sin(q₅)⋅cos(q₁)⋅cos(q₂ + q₃))⋅cos(q₆) + (sin(q₁)⋅cos(q₄) - sin(q₄)⋅sin(q₂ + q₃)⋅cos(q₁))⋅sin(q₆)  -((sin(q₁)⋅sin(q₄) + sin(q₂ + q₃)⋅cos(q₁)⋅cos(q₄))⋅cos(q₅) + sin(q₅)⋅cos(q₁)⋅cos(q₂ + q₃))⋅sin(q₆) + (sin(q₁)⋅cos(q₄) - sin(q₄)⋅sin(q₂ + q₃)⋅cos(q₁))⋅cos(q₆)  -(sin(q₁)⋅sin(q₄) + sin(q₂ + q₃)⋅cos(q₁)⋅cos(q₄))⋅sin(q₅) + cos(q₁)⋅cos(q₅)⋅cos(q₂ + q₃)  -0.303⋅(sin(q₁)⋅sin(q₄) + sin(q₂ + q₃)⋅cos(q₁)⋅cos(q₄))⋅sin(q₅) + (1.25⋅sin(q₂) - 0.054⋅sin(q₂ + q₃) + 1.5⋅cos(q₂ + q₃) + 0.35)⋅cos(q₁) + 0.303⋅cos(q₁)⋅cos(q₅)⋅cos(q₂ + q₃)⎤
⎢                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   ⎥
⎢((sin(q₁)⋅sin(q₂ + q₃)⋅cos(q₄) - sin(q₄)⋅cos(q₁))⋅cos(q₅) + sin(q₁)⋅sin(q₅)⋅cos(q₂ + q₃))⋅cos(q₆) - (sin(q₁)⋅sin(q₄)⋅sin(q₂ + q₃) + cos(q₁)⋅cos(q₄))⋅sin(q₆)  -((sin(q₁)⋅sin(q₂ + q₃)⋅cos(q₄) - sin(q₄)⋅cos(q₁))⋅cos(q₅) + sin(q₁)⋅sin(q₅)⋅cos(q₂ + q₃))⋅sin(q₆) - (sin(q₁)⋅sin(q₄)⋅sin(q₂ + q₃) + cos(q₁)⋅cos(q₄))⋅cos(q₆)  -(sin(q₁)⋅sin(q₂ + q₃)⋅cos(q₄) - sin(q₄)⋅cos(q₁))⋅sin(q₅) + sin(q₁)⋅cos(q₅)⋅cos(q₂ + q₃)  -0.303⋅(sin(q₁)⋅sin(q₂ + q₃)⋅cos(q₄) - sin(q₄)⋅cos(q₁))⋅sin(q₅) + (1.25⋅sin(q₂) - 0.054⋅sin(q₂ + q₃) + 1.5⋅cos(q₂ + q₃) + 0.35)⋅sin(q₁) + 0.303⋅sin(q₁)⋅cos(q₅)⋅cos(q₂ + q₃)⎥
⎢                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   ⎥
⎢                               -(sin(q₅)⋅sin(q₂ + q₃) - cos(q₄)⋅cos(q₅)⋅cos(q₂ + q₃))⋅cos(q₆) - sin(q₄)⋅sin(q₆)⋅cos(q₂ + q₃)                                                                  (sin(q₅)⋅sin(q₂ + q₃) - cos(q₄)⋅cos(q₅)⋅cos(q₂ + q₃))⋅sin(q₆) - sin(q₄)⋅cos(q₆)⋅cos(q₂ + q₃)                                                     -sin(q₅)⋅cos(q₄)⋅cos(q₂ + q₃) - sin(q₂ + q₃)⋅cos(q₅)                                           -0.303⋅sin(q₅)⋅cos(q₄)⋅cos(q₂ + q₃) - 0.303⋅sin(q₂ + q₃)⋅cos(q₅) - 1.5⋅sin(q₂ + q₃) + 1.25⋅cos(q₂) - 0.054⋅cos(q₂ + q₃) + 0.75                       ⎥
⎢                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   ⎥
⎣                                                                             0                                                                                                                                                              0                                                                                                                           0                                                                                                                                   1                                                                                      ⎦
T0_8
⎡-(sin(q₁)⋅sin(q₄) + sin(q₂ + q₃)⋅cos(q₁)⋅cos(q₄))⋅sin(q₅) + cos(q₁)⋅cos(q₅)⋅cos(q₂ + q₃)  ((sin(q₁)⋅sin(q₄) + sin(q₂ + q₃)⋅cos(q₁)⋅cos(q₄))⋅cos(q₅) + sin(q₅)⋅cos(q₁)⋅cos(q₂ + q₃))⋅sin(q₆) - (sin(q₁)⋅cos(q₄) - sin(q₄)⋅sin(q₂ + q₃)⋅cos(q₁))⋅cos(q₆)  ((sin(q₁)⋅sin(q₄) + sin(q₂ + q₃)⋅cos(q₁)⋅cos(q₄))⋅cos(q₅) + sin(q₅)⋅cos(q₁)⋅cos(q₂ + q₃))⋅cos(q₆) + (sin(q₁)⋅cos(q₄) - sin(q₄)⋅sin(q₂ + q₃)⋅cos(q₁))⋅sin(q₆)  -0.303⋅(sin(q₁)⋅sin(q₄) + sin(q₂ + q₃)⋅cos(q₁)⋅cos(q₄))⋅sin(q₅) + (1.25⋅sin(q₂) - 0.054⋅sin(q₂ + q₃) + 1.5⋅cos(q₂ + q₃) + 0.35)⋅cos(q₁) + 0.303⋅cos(q₁)⋅cos(q₅)⋅cos(q₂ + q₃)⎤
⎢                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  ⎥
⎢-(sin(q₁)⋅sin(q₂ + q₃)⋅cos(q₄) - sin(q₄)⋅cos(q₁))⋅sin(q₅) + sin(q₁)⋅cos(q₅)⋅cos(q₂ + q₃)  ((sin(q₁)⋅sin(q₂ + q₃)⋅cos(q₄) - sin(q₄)⋅cos(q₁))⋅cos(q₅) + sin(q₁)⋅sin(q₅)⋅cos(q₂ + q₃))⋅sin(q₆) + (sin(q₁)⋅sin(q₄)⋅sin(q₂ + q₃) + cos(q₁)⋅cos(q₄))⋅cos(q₆)  ((sin(q₁)⋅sin(q₂ + q₃)⋅cos(q₄) - sin(q₄)⋅cos(q₁))⋅cos(q₅) + sin(q₁)⋅sin(q₅)⋅cos(q₂ + q₃))⋅cos(q₆) - (sin(q₁)⋅sin(q₄)⋅sin(q₂ + q₃) + cos(q₁)⋅cos(q₄))⋅sin(q₆)  -0.303⋅(sin(q₁)⋅sin(q₂ + q₃)⋅cos(q₄) - sin(q₄)⋅cos(q₁))⋅sin(q₅) + (1.25⋅sin(q₂) - 0.054⋅sin(q₂ + q₃) + 1.5⋅cos(q₂ + q₃) + 0.35)⋅sin(q₁) + 0.303⋅sin(q₁)⋅cos(q₅)⋅cos(q₂ + q₃)⎥
⎢                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  ⎥
⎢                  -sin(q₅)⋅cos(q₄)⋅cos(q₂ + q₃) - sin(q₂ + q₃)⋅cos(q₅)                                                   -(sin(q₅)⋅sin(q₂ + q₃) - cos(q₄)⋅cos(q₅)⋅cos(q₂ + q₃))⋅sin(q₆) + sin(q₄)⋅cos(q₆)⋅cos(q₂ + q₃)                                                                 -(sin(q₅)⋅sin(q₂ + q₃) - cos(q₄)⋅cos(q₅)⋅cos(q₂ + q₃))⋅cos(q₆) - sin(q₄)⋅sin(q₆)⋅cos(q₂ + q₃)                                                         -0.303⋅sin(q₅)⋅cos(q₄)⋅cos(q₂ + q₃) - 0.303⋅sin(q₂ + q₃)⋅cos(q₅) - 1.5⋅sin(q₂ + q₃) + 1.25⋅cos(q₂) - 0.054⋅cos(q₂ + q₃) + 0.75                       ⎥
⎢                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  ⎥
⎣                                           0                                                                                                                           0                                                                                                                                                             0                                                                                                                                                                     1                                                                                      ⎦


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

First we obtain the position of the wrist center (global reference frame).
#eq

Now we can compute the angles necessary to obtain the given position of the wrist center (theta{1,2,3}).
![alt text][inverse_position]

The angles of the wrist (theta{4,5,6}) to obtain the given orientation of the end-effector are acquired by solving the inverse orientation problem.
![alt text][inverse_orientation]


![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


