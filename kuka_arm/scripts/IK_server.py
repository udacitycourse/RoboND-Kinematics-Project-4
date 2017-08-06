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


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:


        # Forward Kinematics

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
        joint_trajectory_list = []
        for x in range(0, len(req.poses)):
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

            Rrpy = (R_yaw * R_pitch * R_roll* R_URDF_to_DH).evalf()
    
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
            
            R0_3_inv = Transpose(R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3}))
            R3_6 = (R0_3_inv * Rrpy).evalf()

            # print symbolic representation of R3_6, derive the rotation order from that
            # pprint(simplify(T3_4[0:3,0:3] * T4_5[0:3,0:3] * T5_6[0:3,0:3]))
            
            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[1,0]**2 + R3_6[1,1]**2), R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])
	    
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
