#!/usr/bin/env python

from sympy import *
from time import time
from mpmath import radians
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()
    
    ########################################################################################
    ##

    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

    s = {alpha0: 0,     a0: 0,   # d0: N/A    q0: N/A
         alpha1: -pi/2, a1: .35,   d1: .75,   q1: q1,
         alpha2: 0,     a2: 1.25,  d2: 0,     q2: q2-pi/2,
         alpha3: -pi/2, a3: -.054, d3: 0,     q3: q3,
         alpha4: pi/2,  a4: 0,     d4: 1.5,   q4: q4,
         alpha5: -pi/2, a5: 0,     d5: 0,     q5: q5,
         alpha6: 0,     a6: 0,     d6: 0,     q6: q6,
         # alpha7: N/A    a7: N/A
         d7: .303,  q7: 0}


    T0_1 = Matrix([[cos(q1), -sin(q1), 0, a0],
                   [sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
                   [sin(q1)*sin(alpha0), cos(q1)*sin(alpha0), cos(alpha0), cos(alpha0)*d1],
                   [0, 0, 0, 1]])
    T0_1 = T0_1.subs(s)
    
    T1_2 = Matrix([[cos(q2), -sin(q2), 0, a1],
                   [sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
                   [sin(q2)*sin(alpha1), cos(q2)*sin(alpha1), cos(alpha1), cos(alpha1)*d2],
                   [0, 0, 0, 1]])
    T1_2 = T1_2.subs(s)
    
    T2_3 = Matrix([[cos(q3), -sin(q3), 0, a2],
                   [sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                   [sin(q3)*sin(alpha2), cos(q3)*sin(alpha2), cos(alpha2), cos(alpha2)*d3],
                   [0, 0, 0, 1]])
    T2_3 = T2_3.subs(s)
    
    T3_4 = Matrix([[cos(q4), -sin(q4), 0, a3],
                   [sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
                   [sin(q4)*sin(alpha3), cos(q4)*sin(alpha3), cos(alpha3), cos(alpha3)*d4],
                   [0, 0, 0, 1]])
    T3_4 = T3_4.subs(s)
    
    T4_5 = Matrix([[cos(q5), -sin(q5), 0, a4],
                   [sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
                   [sin(q5)*sin(alpha4), cos(q5)*sin(alpha4), cos(alpha4), cos(alpha4)*d5],
                   [0, 0, 0, 1]])
    
    T4_5 = T4_5.subs(s)
    
    T5_6 = Matrix([[cos(q6), -sin(q6), 0, a5],
                   [sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
                   [sin(q6)*sin(alpha5), cos(q6)*sin(alpha5), cos(alpha5), cos(alpha5)*d6],
                   [0, 0, 0, 1]])
    T5_6 = T5_6.subs(s)
    
    T6_G = Matrix([[cos(q7), -sin(q7), 0, a6],
                   [sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
                   [sin(q7)*sin(alpha6), cos(q7)*sin(alpha6), cos(alpha6), cos(alpha6)*d7],
                   [0, 0, 0, 1]])
    T6_G = T6_G.subs(s)
    
    # Create individual transformation matrices
    
    T0_2 = T0_1 * T1_2
    T0_3 = T0_2 * T2_3
    T0_4 = T0_3 * T3_4
    T0_5 = T0_4 * T4_5
    T0_6 = T0_5 * T5_6
    T0_G = T0_6 * T6_G
    
    # Rotate gripper reference frame to coincide with global reference frame
    
    R_z_180 = Matrix([[cos(pi), -sin(pi), 0, 0],
                      [sin(pi), cos(pi),  0, 0],
                      [0,       0,        1, 0],
                      [0,       0,        0, 1]])
    
    R_y_neg90 = Matrix([[cos(-pi/2),  0,  sin(-pi/2), 0],
                        [0,           1,  0,          0],
                        [-sin(-pi/2), 0,  cos(-pi/2), 0],
                        [0,           0,  0,          1]])
    
    # correction rotation from DH to URDF
    # R_corr = simplify(R_z_180 * R_y_neg90)
    
    # for debugging
    # q_subs = {q1: 0.51, q2: -0.61, q3: -2.13, q4: -2.90, q5: -1.49, q6: 0.56}
    # print("T0_1 = ", T0_1.evalf(subs=q_subs))
    # print("T0_2 = ", T0_2.evalf(subs=q_subs))
    # print("T0_3 = ", T0_3.evalf(subs=q_subs))
    # print("T0_4 = ", T0_4.evalf(subs=q_subs))
    # print("T0_5 = ", T0_5.evalf(subs=q_subs))
    # print("T0_6 = ", T0_6.evalf(subs=q_subs))
    # print("T0_G = ", T0_G.evalf(subs=q_subs))
    
    # T_total = T0_G * R_corr
    
    # Extract rotation matrices from the transformation matrices
    # The rotation matrix is defined as the 3x3 submatrix beginning
    # at (0, 0) for each homogeneous transform.
    ###
    
    R0_1 = T0_1[0:3,0:3]
    R0_2 = T0_2[0:3,0:3]
    R0_3 = T0_3[0:3,0:3]
    R0_4 = T0_4[0:3,0:3]
    R0_5 = T0_5[0:3,0:3]
    R0_6 = T0_6[0:3,0:3]
    R0_G = T0_G[0:3,0:3]
    
    
    ## Insert IK code here!
    x = 0
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
    
    R_yaw = Matrix([[cos(yaw), -sin(yaw), 0],
                    [sin(yaw), cos(yaw), 0],
                    [0, 0, 1]])
    R_pitch = Matrix([[cos(pitch), 0, sin(pitch)],
                      [0, 1, 0],
                      [-sin(pitch), 0, cos(pitch)]])
    R_roll = Matrix([[1, 0, 0],
                     [0, cos(roll), -sin(roll)],
                     [0, sin(roll), cos(roll)]])
    
    R_y_90 = Matrix([[cos(pi/2), 0, sin(pi/2)],
                     [0, 1, 0],
                     [-sin(pi/2), 0, cos(pi/2)]])

    R_z_180 = Matrix([[cos(pi), -sin(pi), 0],
                      [sin(pi), cos(pi),  0],
                      [0,       0,        1]])

    
    R_URDF_to_DH = R_y_90 * R_z_180
    
    Rrpy = R_yaw * R_pitch * R_roll * R_URDF_to_DH
    
    nx = Rrpy[0,2]
    ny = Rrpy[1,2]
    nz = Rrpy[2,2]
    
    wx = (px - d7 * nx)
    wx = wx.subs(s)

    
    wy = py - d7 * ny
    wy = wy.subs(s)
    
    wz = pz - d7 * nz
    wz = wz.subs(s)
    
    # Calculate joint angles using Geometric IK method
    #
    #
    ###

    theta1 = 0
    theta2 = 0
    theta3 = 0
    theta4 = 0
    theta5 = 0
    theta6 = 0

    # triangle sides
    A = a2
    B = sqrt(a3**2 + d4**2)
    C = sqrt((sqrt(wx**2 + wy**2)-a1)**2 + (wz - d1)**2)
    
    theta1 = atan2(wy, wx).evalf(subs=s)
    
    cos_gamma = (A**2 + C**2 - B**2) / (2 * A * C)
    
    gamma = atan2(sqrt(1-cos_gamma**2), cos_gamma)

    theta2 = (pi/2 - gamma - atan2(wz-d1, sqrt(wx**2 + wy**2) - a1)).subs(s)
    
    cos_beta = (A**2 + B**2 - C**2) / (2 * A * B)
    
    beta = atan2(sqrt(1-cos_beta**2), cos_beta)

    
    theta3 = (pi/2 - beta - atan2(abs(a3), d4)).subs(s)
    
    R0_3_inv = R0_3.inv("LU")
    R3_6 = R0_3_inv * Rrpy
    # R3_6 = R3_6.subs(s)
    R3_6 = R3_6.subs({q1: theta1, q2: theta2, q3: theta3})
    print(R3_6)
    
    theta5 = atan2(-R3_6[2,0], sqrt(R3_6[0,0]**2 + R3_6[1,0]**2))
    theta4 = atan2(R3_6[2,1], R3_6[2,2])
    theta6 = atan2(R3_6[1,0], R3_6[0,0])

    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!

    q_subs = {q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6}
    T0_G = T0_G.subs(q_subs)

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [wx,wy,wz] # <--- Load your calculated WC values in this array
    your_ee = [T0_G[0,3], T0_G[1,3], T0_G[2,3]] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
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
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 3

    test_code(test_cases[test_case_number])
