#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray

def publish_test_messages():
    rospy.init_node('test_publisher', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz
    rospy.loginfo('Test running')

    front_left_pub = rospy.Publisher('/front_left_leg_target', Float64MultiArray, queue_size=10)
    back_left_pub = rospy.Publisher('/back_left_leg_target', Float64MultiArray, queue_size=10)
    front_right_pub = rospy.Publisher('/front_right_leg_target', Float64MultiArray, queue_size=10)
    back_right_pub = rospy.Publisher('/back_right_leg_target', Float64MultiArray, queue_size=10)

    while not rospy.is_shutdown():
        front_left_pub.publish(Float64MultiArray(data=[20, 60, 20]))
        back_left_pub.publish(Float64MultiArray(data=[25, 35, 20]))
        front_right_pub.publish(Float64MultiArray(data=[30, 40, 25]))
        back_right_pub.publish(Float64MultiArray(data=[35, 45, 25]))
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_test_messages()
    except rospy.ROSInterruptException:
        pass
