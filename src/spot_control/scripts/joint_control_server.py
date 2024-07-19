#!/usr/bin/env python3

import rospy
from spot_control.srv import jointControl, jointControlResponse
from sensor_msgs.msg import JointState


# rosservice call /joint_control "[0.4, -1.6, 0.1, 0.1, -1.5, 0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]"
# rosservice call /joint_control "[0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]"

# rosservice call /joint_control "[90.0, 152.73395554926716, 121.36697777463375, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]"

class JointControlServer:
    def __init__(self):
        rospy.init_node('joint_control_server')
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.joint_state = JointState()
        self.joint_state.name = [
            'front_left_shoulder',
            'front_left_leg',
            'front_left_foot',
            'front_right_shoulder',
            'front_right_leg',
            'front_right_foot',
            'rear_left_shoulder',
            'rear_left_leg',
            'rear_left_foot',
            'rear_right_shoulder',
            'rear_right_leg',
            'rear_right_foot'
        ]

        self.srv = rospy.Service('joint_control', jointControl, self.handle_joint_control)

    def handle_joint_control(self, req):
        rospy.loginfo(f"Received joint angles: {req.joint_angles}")
        self.joint_state.position = req.joint_angles
        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_state_pub.publish(self.joint_state)
        return jointControlResponse(success=True, message="Joint angles updated")

if __name__ == "__main__":
    JointControlServer()
    rospy.spin()

