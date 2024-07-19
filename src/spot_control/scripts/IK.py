#!usr/bin/env python3

import rospy
import numpy as np

def IK(x, y, z):
    # I think these are the correct leg lengths
    l1 = 20
    l2 = 80
    l3 = 80
    x = -x

    D = np.sqrt((z**2 + y**2) - l1**2)
    G = np.sqrt(D**2 + x**2)

    # hip angle
    omega = np.arctan2(z, y) + np.arctan2(D, l1)

    # ankle angle
    phi = np.arccos((G**2 - l2**2 - l3**2) / (2 * l2 * l3))

    # knee angle
    theta = np.arctan2(x, D) + np.arcsin((l3 * np.sin(phi)) / G)

    omega = (omega) - (np.pi / 2)
    if omega < -0.55 or omega > 0.55:
        rospy.loginfo('Shoulder angle out of range')
    
    theta = (theta)
    if theta < -2.67 or theta > 1.55:
        rospy.loginfo('Knee angle out of range')

    phi = -phi 
    if phi < -2.6 or phi > 0.1:
        rospy.loginfo('Ankle angle out of range')

    joint_angles = [omega, theta, phi]

    return joint_angles

## Z needs to be negative for the right side
print(IK(20, 100, 40))
