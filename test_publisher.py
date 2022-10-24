#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String, Float64MultiArray

# def talker():
#     pub = rospy.Publisher('apriltag_position', Float64MultiArray, queue_size=10)
#     rospy.init_node('apriltag_publisher', anonymous=True)
#     rate = rospy.Rate(2) #10Hz

#     while not rospy.is_shutdown():
#         data = Float64MultiArray()
#         array = [np.sin(rospy.get_time()/5), -np.cos(rospy.get_time()/5)]
#         data.data = array
#         rospy.loginfo(data)
#         pub.publish(data)
#         rate.sleep()


# def talker():
#     pub = rospy.Publisher('imu_sensor', String, queue_size= 10)
#     rospy.init_node('imu_publisher', anonymous=True)
#     rate = rospy.Rate(10) #10Hz

#     while not rospy.is_shutdown():
#         data = "1.00 2.00 3.00\r\n"
#         rospy.loginfo(data)
#         pub.publish(data)
#         rate.sleep()

a = 0
b = 10

def talker():
    global a, b
    pub = rospy.Publisher("test", String, queue_size=10)
    rospy.init_node("test_publisher", anonymous=True)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        data = "%s %s"%(a,b)
        rospy.loginfo(data)
        pub.publish(data)
        a += 1
        b += 2
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass

