#!/usr/bin/env python3

import rospy
from spot_control.srv import jointControl

def send_joint_angles(joint_angles):
    rospy.wait_for_service('joint_control')
    try:
        joint_control = rospy.ServiceProxy('joint_control', jointControl)
        response = joint_control(joint_angles)
        if response.success:
            rospy.loginfo("Joint angles updated successfully.")
        else:
            rospy.logwarn("Failed to update joint angles: %s", response.message)
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return False

if __name__ == "__main__":
    rospy.init_node('joint_control_client')
    rospy.loginfo('Joint Control Client Started')
    
    # Example joint angles for all legs and joints
    joint_angles = [0.1, 0.2, 0.3] * 4  # Replace with actual target angles

    success = send_joint_angles(joint_angles)
    if success:
        rospy.loginfo("Joint angles updated successfully.")
    else:
        rospy.logwarn("Failed to update joint angles.")

