#!/usr/bin/env python3
# license removed for brevity
import rospy
import sys
import numpy as np
from std_msgs.msg import Float64

joint_angle_float = 0.0

def updateJointCallback(msg):
    global joint_angle_float
    rospy.loginfo("Receieved new angle: %f", msg.data)
    joint_angle_float = msg.data

def setJoint():
    # Declares that this function uses the global variable that can be modified.
    global joint_angle_float
    pub = rospy.Publisher('/spotmicroai/rear_left_leg_position_controller/command', Float64, queue_size=10)
    rospy.Subscriber('/example_topic', Float64, updateJointCallback)
    rate = rospy.Rate(10) # 10 Hz
    joint_angle_float = np.radians(joint_angle_float)
    while not rospy.is_shutdown():
        pub.publish(joint_angle_float)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node("set_joints", anonymous=True)
        rospy.loginfo('Running basic control example')
        # For this example, I just have the starting angle be whatever is provided from command
        # line arguement. In the future, we could add a config file that has starting angles for
        # each joint (prob 0 for home config).
        setJoint()
        rospy.sleep(1)
    except rospy.ROSInterruptException:
        pass