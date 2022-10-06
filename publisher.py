#!/usr/bin/env python

import rospy
from std_msgs.msg import String
 
def apriltag_publisher(x_pos, y_pos, Hz, LOG_INFO = True):
    pub = rospy.Publisher('apriltag_position', String, queue_size=10)
    rospy.init_node('apriltag_publisher', anonymous=True)
    rate = rospy.Rate(Hz)

    pos = "%s %s" % (x_pos, y_pos)
    if LOG_INFO:rospy.loginfo(pos)
    pub.publish(pos)
    rate.sleep()

if __name__ == '__main__':
    try:
        apriltag_publisher()
    except rospy.ROSInterruptException:
        pass