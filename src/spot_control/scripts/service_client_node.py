#!/usr/bin/env python3

import rospy
from spot_control.srv import multiplier

def multiplier_client(x,y):
    rospy.init_node('client_node')
    # Waits for service to exist in the ROS core
    rospy.wait_for_service('multiplier') # takes in the name of the service
    rate = rospy.Rate(1) # 1 second

    while not rospy.is_shutdown():
        try:
            #create object for the service
            multiply_two_ints = rospy.ServiceProxy('multiplier', multiplier) # takes in name and type of service
            #call the service
            response = multiply_two_ints(x, y)
            rospy.loginfo(response.result)
            rate.sleep()

        except rospy.ServiceException as e:
            print('Service call failed %s', e)

if __name__ == '__main__':
    multiplier_client(7, 2)