#!/usr/bin/env python3

import rospy
import sys

from std_msgs.msg import Float64

def joint_angle_publisher(new_angle):
    rospy.init_node('joint_angle_publisher', anonymous=True)
    pub = rospy.Publisher('/example_topic', Float64, queue_size=10)
    rospy.sleep(0.1)
    pub.publish(new_angle)
    rospy.loginfo("Set joint angle to: %f", new_angle)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        rospy.logerr("Usage: rosrun spot_control JointUpdate.py <joint_angle_deg>")
        sys.exit(1)
    try:
        joint_angle = float(sys.argv[1])
        joint_angle_publisher(joint_angle)
    except rospy.ROSInterruptException:
        pass
