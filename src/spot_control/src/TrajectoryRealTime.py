#!/usr/bin/env python3
# license removed for brevity
import rospy
import numpy as np

def runTest():
    rospy.init_node("test_timer")
    now = rospy.get_rostime()
    rospy.loginfo("Current Time: %i %i", now.secs, now.nsecs)

    rospy.sleep(2.5)

    time_secs = rospy.get_rostime().secs - now.secs
    time_nsecs = rospy.get_rostime().nsecs - now.nsecs
    rospy.loginfo("Slept for: %i", time_secs)  # 2 s
    rospy.loginfo("Slept for: %i", time_nsecs*(10.0**-6.0))  # 506 ms


if __name__ == '__main__':
    try:
        pass
    except rospy.ROSInterruptException:
        pass