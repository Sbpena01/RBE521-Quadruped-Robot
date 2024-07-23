#!/usr/bin/env python3

import rospy
import math
import numpy as np
from std_msgs.msg import Float64MultiArray



def IK(x, y, z):
    # I think these are the correct leg lengths
    l1 = 2.0
    l2 = 8.0
    l3 = 8.0

    D = math.sqrt((z**2 + y**2) - l1**2)
    G = math.sqrt(D**2 + x**2)

    # hip angle
    omega = math.atan2(y, z) + math.atan2(D, l1)
    # ankle angle
    phi = math.acos((G**2 - l2**2 - l3**2) / (2 * l2 * l3))
    # knee angle
    theta = math.atan2(x, D) + math.asin((l3 * math.sin(phi)) / G)

    omega = (omega) + (math.pi / 2)
    # if omega < -0.55 or omega > 0.55:
    #     rospy.loginfo('Shoulder angle out of range')
    #     pass
    
    # theta = (theta)
    # if theta < -2.67 or theta > 1.55:
    #     rospy.loginfo('Knee angle out of range')

    # phi = -phi 
    # if phi < -2.6 or phi > 0.1:
    #     rospy.loginfo('Ankle angle out of range')

    joint_angles = [-omega, -theta, -phi]

    return joint_angles


def front_left_leg_callback(msg):
    x = msg.data[0]
    y = msg.data[1]
    z = msg.data[2]
    # rospy.loginfo("%f, %f, %f", x, y, z)
    [x, y, z] = transform(x, y, z)

    joint_angles = IK(x, y, z)
    joint_angles[0] = 0
    joint_angles_msg = Float64MultiArray()
    joint_angles_msg.data = joint_angles
    #rospy.loginfo('Successfully published FL angles: %s', joint_angles_msg.data)
    front_left_leg_pub.publish(joint_angles_msg)

def back_left_leg_callback(msg):
    x = msg.data[0]
    y = msg.data[1]
    z = msg.data[2]
    [x, y, z] = transform(x, y, z)
    joint_angles = IK(x, y, z)
    joint_angles[0] = 0
    joint_angles_msg = Float64MultiArray()
    joint_angles_msg.data = joint_angles
    #rospy.loginfo('Successfully published BL angles: %s', joint_angles_msg.data)
    back_left_leg_pub.publish(joint_angles_msg)


## TODO Check if z needs to be flipped

def front_right_leg_callback(msg):
    x = msg.data[0]
    y = msg.data[1]
    z = msg.data[2]
    [x, y, z] = transform(x, y, z)
    joint_angles = IK(x, y, z)
    joint_angles[0] = 0
    joint_angles_msg = Float64MultiArray()
    joint_angles_msg.data = joint_angles
    #rospy.loginfo('Successfully published FR angles: %s', joint_angles_msg.data)
    front_right_leg_pub.publish(joint_angles_msg)

def back_right_leg_callback(msg):
    x = msg.data[0]
    y = msg.data[1]
    z = msg.data[2]
    [x, y, z] = transform(x, y, z)
    joint_angles = IK(x, y, z)
    joint_angles[0] = 0
    joint_angles_msg = Float64MultiArray()
    joint_angles_msg.data = joint_angles

    #rospy.loginfo('Successfully published BR angles: %s', joint_angles_msg.data)
    back_right_leg_pub.publish(joint_angles_msg)

def transform(Px, Py, Pz):
    current_frame = np.transpose(np.array([Px, Py, Pz, 1]))
    transformation = np.array([[-1, 0, 0, 0], [0, 1, 0, 2.0],[0, 0, -1, 8*math.sqrt(2)],[0, 0, 0, 1]])
    new_frame = np.matmul(transformation, current_frame)
    # print(new_frame)
    # rospy.loginfo("Transformed coords: %f, %f, %f", new_frame[0], new_frame[1], new_frame[2])
    return [new_frame[0], new_frame[1], new_frame[2]]

if __name__ == '__main__':
    try:
        rospy.init_node('inverse_kinematics', anonymous=True)
        rospy.loginfo('Inverse kinematics running.')

        # Publish the joint angles to the corresponding legs
        front_left_leg_pub = rospy.Publisher('/front_left_leg_angles', Float64MultiArray, queue_size=10)
        back_left_leg_pub = rospy.Publisher('/rear_left_leg_angles', Float64MultiArray, queue_size=10)
        front_right_leg_pub = rospy.Publisher('/front_right_leg_angles', Float64MultiArray, queue_size=10)
        back_right_leg_pub = rospy.Publisher('/rear_right_leg_angles', Float64MultiArray, queue_size=10)

        # Subscribe to the target foot positions from the trajectories
        rospy.Subscriber('/front_left_leg_target', Float64MultiArray, front_left_leg_callback)
        rospy.Subscriber('/back_left_leg_target', Float64MultiArray, back_left_leg_callback)
        rospy.Subscriber('/front_right_leg_target', Float64MultiArray, front_right_leg_callback)
        rospy.Subscriber('/back_right_leg_target', Float64MultiArray, back_right_leg_callback)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass