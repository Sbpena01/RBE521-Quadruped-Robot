#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray

def front_left_leg_callback(msg):
    rospy.loginfo('Received Front Left Leg Angles: %s', msg.data)

def back_left_leg_callback(msg):
    rospy.loginfo('Received Back Left Leg Angles: %s', msg.data)

def front_right_leg_callback(msg):
    rospy.loginfo('Received Front Right Leg Angles: %s', msg.data)

def back_right_leg_callback(msg):
    rospy.loginfo('Received Back Right Leg Angles: %s', msg.data)

if __name__ == '__main__':
    try:
        rospy.init_node('test_joint_angles_subscriber', anonymous=True)
        rospy.loginfo('Test node for joint angles subscriber running.')

        # Subscribe to the joint angles topics
        rospy.Subscriber('front_left_leg_angles', Float64MultiArray, front_left_leg_callback)
        rospy.Subscriber('back_left_leg_angles', Float64MultiArray, back_left_leg_callback)
        rospy.Subscriber('front_right_leg_angles', Float64MultiArray, front_right_leg_callback)
        rospy.Subscriber('back_right_leg_angles', Float64MultiArray, back_right_leg_callback)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
