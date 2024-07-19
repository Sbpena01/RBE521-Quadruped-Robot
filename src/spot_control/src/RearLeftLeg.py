#!/usr/bin/env python3
# license removed for brevity
import rospy
import sys
import numpy as np
from std_msgs.msg import Float64MultiArray, Float64

shoulder = 0.0
foot = 0.0
leg = 0.0

def updateJointCallback(msg):
    global shoulder, leg, foot
    rospy.loginfo("Receieved news angle: %f, %f, %f", msg.data[0], msg.data[1], msg.data[2])
    [shoulder, leg, foot] = msg.data

def setJoints():
    # Declares that this function uses the global variable that can be modified.
    global shoulder, leg, foot

    shoulder_pub = rospy.Publisher('/spotmicroai/rear_left_shoulder_position_controller/command',
                                   Float64, queue_size=10)
    leg_pub = rospy.Publisher('/spotmicroai/rear_left_leg_position_controller/command',
                              Float64, queue_size=10)
    foot_pub = rospy.Publisher('/spotmicroai/rear_left_foot_position_controller/command',
                               Float64, queue_size=10)
    rospy.Subscriber('/rear_left_leg_angles', Float64MultiArray, updateJointCallback)  # TODO replace topic

    rate = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():
        shoulder_pub.publish(shoulder)
        leg_pub.publish(leg)
        foot_pub.publish(foot)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node("rear_left_leg_joints", anonymous=True)
        rospy.loginfo('Running joint controller for front left leg')
        # For this example, I just have the starting angle be whatever is provided from command
        # line arguement. In the future, we could add a config file that has starting angles for
        # each joint (prob 0 for home config).
        setJoints()
        rospy.sleep(1)
    except rospy.ROSInterruptException:
        pass