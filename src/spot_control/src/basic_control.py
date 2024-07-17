#!/usr/bin/env python3
# license removed for brevity
import rospy
import numpy as np

from std_msgs.msg import String
from gazebo_msgs.srv import ApplyJointEffort, GetJointProperties

jointName = 'front_left_shoulder'
startTime = rospy.Duration.from_sec(0)
duration = rospy.Duration.from_sec(0.01)

reference_angle = -20
effort = 0
Kp = 300

def setEffort(effort_to_add):
    # rospy.wait_for_service('/gazebo/apply_joint_effort')
    move = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
    try:
        move(jointName, (effort + effort_to_add), startTime, duration)
        rospy.loginfo("Move was successful.")
    except rospy.ServiceException as e:
        rospy.logerr("Move failed.")
        # print("Service call failed: %s"%e)

def getPosition():
    joint = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
    angle = np.degrees(joint(jointName).position)
    rospy.loginfo(f"Current position of front_left_shoulder: {angle}")
    return angle
    

def pController():
    while True:
        actual_angle = getPosition()
        error = reference_angle - actual_angle
        setEffort(Kp * error)
        rospy.sleep(0.01)


if __name__ == '__main__':
    rospy.init_node('simple_P_controller')
    rospy.wait_for_service('/gazebo/apply_joint_effort')
    rospy.wait_for_service('/gazebo/get_joint_properties')
    try:
        pController()
    except rospy.ROSInterruptException:
        pass