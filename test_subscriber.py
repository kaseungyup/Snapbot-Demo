#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String, Float64MultiArray

acc_x = 0.0
acc_y = 0.0
acc_z = 0.0

# x = 0.0
# y = 0.0

def callback(data):
    # global acc_x, acc_y, acc_z
    # array = data.data.split()
    # acc_x = float(array[0])
    # acc_y = float(array[1])
    # acc_z = float(array[2])
    # print(acc_x, acc_y, acc_z)
    print(data)
   
# def get_acc():
#     return acc_x, acc_y, acc_z

# def callback2(data):
#     global x,y
#     x = data.data[0]
#     y = data.data[1]
#     print(x, y)

if __name__ == '__main__':
    rospy.init_node('subscriber', anonymous=True)
    rospy.Subscriber("imu_sensor", String, callback)
    # rospy.Subscriber("apriltag_position", Float64MultiArray, callback2)
    rospy.spin()
