#!/usr/bin/env python
#-*- coding: utf-8 -*-
"""
Created on Wed Mar  6 12:27:43 2019

@author: mattj
"""

" Inverse Kinematics for Baxter Arm"

import math as m
import numpy as np
from sensor_msgs.msg import JointState
from kinematic_engine.srv import *
import rospy


def inverse_kinematics_arm(x04,y04,z04):

    L0 = 0.27035  # mm
    L1 = 0.069
    L2 = 0.36435
    L3 = 0.069
    L4 = 0.37429
    L5 = 0
    L6 = 0.36830
    Lh = m.sqrt(L2**2 + L3**2)

    #'T06 = T01*T12*T23*T34*T45*T56'
    #'T04 = T01*T12*T23*T34'
    #'T36 = T34*T45*T56'

    " Finding Theta 1"

    #x04 = T04[0,3]
    #y04 = T04[1,3]
    #z04 = T04[2,3]

    Theta_1 = m.atan2(y04,x04)

    "Finding Theta_2"
    " Finding Coefficients and Solving QUadratic"

    E = 2*Lh*(L1 - (x04/m.cos(Theta_1)))
    F = 2*Lh*z04
    G = (x04 / m.cos(Theta_1))**2 + L1**2 + Lh**2 - L4**2 + z04**2 - 2*((L1*x04)/m.cos(Theta_1))
    num = m.sqrt(E**2 + F**2 - G**2)
    den = (G - E)
    t12_up = (-F + num)/den
    t12_down = (-F - num)/den
    #t12 = -F + m.sqrt((E +F -G)**2) / (G-E)

    Theta_2_up = 2*m.atan(t12_up)
    Theta_2_down = 2*m.atan(t12_down)

    #Theta_2 = 20*(m.pi/180) # remove when theta2 is fixed"

    "Finding Theta 4: "

    Y = -z04 - (Lh*m.sin(Theta_2_up))
    X = (x04/m.cos(Theta_1)) - L1 - Lh*m.cos(Theta_2_up)

    Theta_4 = m.atan2(Y,X) - Theta_2_up

    "Finding Theta 5-7"
    " Forward Kinematics Assumptions Should be Imported here"

    " Initial Conditions"

    " Denavit HartenBerg Conditions for Baxter"
    "%           Theta              di         ai       alpha "
    joint1 = np.array([ Theta_1,    0,      0,           0])
    joint2 = np.array([ Theta_2_up,    0,     L1,     -(m.pi/2)])
    joint4 = np.array([ Theta_4 + (m.pi/2),       0,      Lh,          0])
  #  joint5 = np.array([ theta_5,   L4,      0,        (m.pi/2)])
  #  joint6 = np.array([ theta_6,                0,       L5,       -(m.pi/2)])
  #  joint7 = np.array([ theta_7,                0,      0,          (m.pi/2)  ])


    def dhMatrix(t,d,r,a): # theta, d, a, alpha
        """Gets the Denavit-Hartenberg parameters and returns a numpy-based Denavit-Hartenberg matrix"""
        M = np.matrix([
        [m.cos(t) ,-m.sin(t) , 0 , r  ],
        [m.sin(t)*m.cos(a) , m.cos(t)*m.cos(a) ,-m.sin(a) , -d*m.sin(a)  ],
        [m.sin(t)*m.sin(a),  m.cos(t)*m.sin(a), m.cos(a),  d*m.cos(a)        ],
        [0,         0,                  0,                  1             ]])

        return M

        " Computing the Homogeneous Matrix"
    T01 = dhMatrix(theta_1,0,0,0)
    T12 = dhMatrix(theta_2,0,L1,joint2[3])
    T24 = dhMatrix(joint4[0],joint4[1],joint4[2],joint4[3])
    #T34 = dhMatrix(joint4[0],joint4[1],joint4[2],joint4[3])
    #T45 = dhMatrix(joint5[0],joint5[1],joint5[2],joint5[3])
    #T56 = dhMatrix(joint6[0],joint6[1],joint6[2],joint6[3])
    T04 = T01*T12*T24

    R33 = T04[2,2]
    R13 = T04[0,2]
    R22 = T04[1,1]
    R21 = T04[1,0]
    R23 = T04[1,2]
    return joint1, joint2, 0, joint4, R33, R13, R22, R21, R23

def Inverse_Kinematics_wrist(R33,R13,R22,R21,R23):


    Theta_5 = m.atan2(R33,R13)

    "Finding Theta_7"

    #R22 = T36[1,1];
    #R21 = T36[1,0];

    Theta_7 = m.atan2(-R22,R21)

    " FInding Theta_6"

    R21 = R21 / m.cos(Theta_7)
    #R23 = T36[1,2]

    Theta_6 = m.atan2(R21,-R23)
    return Theta_5, Theta_6, Theta_7



def get_kinematic(req):
    
    pose = req.pose

    joint1, joint2, joint3, joint4, R33, R13, R22, R21, R23 = inverse_kinematics_arm(pose.position.x,pose.position.y,pose.position.z)
    joint5, joint6, joint7 = Inverse_Kinematics_wrist(R33,R13,R22,R21,R23)
    joints = JointState()
    joints.header.stamp = rospy.Time.now()
    joints.name = ['joint0', 'joint1', 'joint2', 'joint3']
    joints.position = [joint1, joint2, joint3, joint4, joint5, joint6, joint7]
    joints.velocity = []
    joints.effort = []

    return kinematic_engine.srv.GetIKResponse(joints)
    
    
    
def server():
    rospy.init_node('string')
    s = rospy.Service ('/get_ik',GetIK, get_kinematic)
    rospy.spin()
        
if __name__ == "__main__":
    server()




" BEGIN INVERSE KINEMATICS" 

