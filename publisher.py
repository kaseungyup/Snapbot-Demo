#!/usr/bin/env python

import rospy
from std_msgs.msg import String
 
def x_publisher(x_val, Hz):
    pub = rospy.Publisher('acc_x', String, queue_size=10)
    rospy.init_node('xy_publisher', anonymous=True)
    rate = rospy.Rate(Hz) # 50hz

    acc_x = "%s" % (x_val)
    rospy.loginfo(acc_x)
    pub.publish(acc_x)
    rate.sleep()

if __name__ == '__main__':
    try:
        x_publisher()
    except rospy.ROSInterruptException:
        pass

def xy_publisher(x_pos, y_pos, Hz):
    pub = rospy.Publisher('pos', String, queue_size=10)
    rospy.init_node('xy_publisher', anonymous=True)
    rate = rospy.Rate(Hz) # 50hz

    pos = "%s %s" % (x_pos, y_pos)
    rospy.loginfo(pos)
    pub.publish(pos)
    rate.sleep()

if __name__ == '__main__':
    try:
        xy_publisher()
    except rospy.ROSInterruptException:
        pass