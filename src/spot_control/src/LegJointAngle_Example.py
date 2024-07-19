#!/usr/bin/env python3

import rospy
import sys

from std_msgs.msg import Float64MultiArray

def joint_angle_publisher(angle1, angle2, angle3):
    rospy.init_node('leg_joint_angle_example', anonymous=True)
    pub = rospy.Publisher('/front_left_leg_angles', Float64MultiArray, queue_size=10)
    rospy.sleep(0.5)
    msg = Float64MultiArray()
    msg.data = [float(angle1), float(angle2), float(angle3)]
    rospy.loginfo("Set joint angles to: %f, %f, %f", msg.data[0], msg.data[0], msg.data[0])
    pub.publish(msg)

        
    

if __name__ == '__main__':
    if len(sys.argv) < 4:
        rospy.logerr("Usage: rosrun spot_control LegJointAngle_Example.py <angle1> <angle2> <angle3>")
        sys.exit(1)
    else:
        angle1 = float(sys.argv[1])
        angle2 = float(sys.argv[2])
        angle3 = float(sys.argv[3])
        joint_angle_publisher(angle1, angle2, angle3)
    try:
        rospy.loginfo("Spining...")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
