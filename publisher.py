#!/usr/bin/env python

import rospy
from std_msgs.msg import String
 
def apriltag_publisher(x_pos, y_pos, yaw, Hz, LOG_INFO = True):
    pub = rospy.Publisher('apriltag_position', String, queue_size=10)
    rospy.init_node('apriltag_publisher', anonymous=True)
    rate = rospy.Rate(Hz)

    pos = "%s %s %s" % (x_pos, y_pos, yaw)
    if LOG_INFO:rospy.loginfo(pos)
    pub.publish(pos)
    rate.sleep()

def flag_publisher(flag):
    pub = rospy.Publisher('flag', String, queue_size=10)
    rospy.init_node('flag_publisher', anonymous=True)

    msg = "%s" % flag
    pub.publish(msg)
    rospy.loginfo(msg)



# if __name__ == '__main__':
#     try:
#         flag_publisher(1)
#     except rospy.ROSInterruptException:
#         pass
