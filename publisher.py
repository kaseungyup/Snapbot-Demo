#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64MultiArray
 
def x_publisher(x_val, Hz):
    pub = rospy.Publisher('acc_x', String, queue_size=10)
    rospy.init_node('xy_publisher', anonymous=True)
    rate = rospy.Rate(Hz) # 50hz

    acc_x = "%f" % (x_val)
    rospy.loginfo(acc_x)
    pub.publish(acc_x)
    rate.sleep()

if __name__ == '__main__':
    try:
        x_publisher()
    except rospy.ROSInterruptException:
        pass

def apriltag_publisher(x_pos, y_pos, Hz):
    pub = rospy.Publisher('apriltag_position', Float64MultiArray, queue_size=10)
    rospy.init_node('apriltag_publisher', anonymous=True)
    rate = rospy.Rate(Hz)

    while not rospy.is_shutdown():
        pos = Float64MultiArray()
        array = [x_pos, y_pos]
        pos.data = array
        rospy.loginfo(pos.data)
        pub.publish(pos)
        rate.sleep()

if __name__ == '__main__':
    try:
        apriltag_publisher()
    except rospy.ROSInterruptException:
        pass