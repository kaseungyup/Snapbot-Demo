#!/usr/bin/env python

import rospy
from std_msgs.msg import String
 
def xy_publisher(x_pos, y_pos, Hz):
    pub = rospy.Publisher('chatter', String, queue_size=10)
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