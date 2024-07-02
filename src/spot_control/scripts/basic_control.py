#!/usr/bin/env python
# license removed for brevity
import rospy
import gazebo_msgs
import gazebo_plugins
import gazebo_ros
from std_msgs.msg import String

def client():
    rospy.wait_for_service('gazebo/ApplyJointEffort')
    try:
        add_two_ints = rospy.ServiceProxy('gazebo/ApplyJointEffort', AddTwoInts)
        resp1 = add_two_ints(x, y)
        return resp1.sum
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass